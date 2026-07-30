#pragma once
#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <map>
#include <sstream>
#include <vector>
using namespace std;

// Aggregates the per-sensor scores from the blackboard into the 3-layer model
// (sensor -> 8-sector coverage -> capability) and publishes:
//   /monitor/scores : full score set for the MRM supervisor
//   /diagnostics    : per-sensor DiagnosticStatus for the Autoware chain
// sensor_score = timeout_score (from the FTTI checkers) * quality_score (from
// the quality checkers). Sector fusion: 1 - prod(1 - s_i) over covering lidars.
class PublishScores : public BT::SyncActionNode
{
public:
    static constexpr const char* SECTOR_LABELS[8] =
        {"F", "FL", "L", "RL", "B", "RR", "R", "FR"};  // 45 deg each, CCW from +x

    struct SensorEntry {
        std::string name;
        std::string type;      // "lidar" | "camera"
        double yaw_deg = 0.0;
        double fov_deg = 360.0;
    };

    PublishScores(const std::string& name, const BT::NodeConfig& conf,
                  const BT::RosNodeParams& params)
                  : SyncActionNode(name, conf)
    {
        node_ = params.nh.lock();
        for (const std::string& type : {std::string("lidar"), std::string("camera")}) {
            std::vector<std::string> names;
            node_->get_parameter("sensors." + type + ".names", names);
            for (const auto& sensor_name : names) {
                const std::string prefix = "sensors." + type + "." + sensor_name + ".";
                SensorEntry e;
                e.name = sensor_name;
                e.type = type;
                if (!node_->has_parameter(prefix + "yaw_deg"))
                    node_->declare_parameter<double>(prefix + "yaw_deg", 0.0);
                node_->get_parameter(prefix + "yaw_deg", e.yaw_deg);
                node_->get_parameter(prefix + "h_fov_deg", e.fov_deg);
                sensors_.push_back(e);
            }
        }
        scores_pub_ = node_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
            "/monitor/scores", 10);
        diag_pub_ = node_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
            "/diagnostics", 10);
        if (!node_->has_parameter("diag_recovery_hold_s"))
            node_->declare_parameter<double>("diag_recovery_hold_s", 3.0);
        node_->get_parameter("diag_recovery_hold_s", recovery_hold_s_);
        if (!node_->has_parameter("diag_recovery_margin"))
            node_->declare_parameter<double>("diag_recovery_margin", 0.2);
        node_->get_parameter("diag_recovery_margin", recovery_margin_);
        if (!node_->has_parameter("warmup_timeout_s"))
            node_->declare_parameter<double>("warmup_timeout_s", 120.0);
        node_->get_parameter("warmup_timeout_s", warmup_timeout_s_);
        warmup_start_ = std::chrono::steady_clock::now();
    }

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override
    {
        auto bb = config().blackboard;
        auto get_d = [&](const std::string& key, double def) {
            double v = def;
            (void)bb->get(key, v);
            return v;
        };

        // --- layer 1: per-sensor scores ---
        std::vector<double> scores(sensors_.size());
        std::vector<double> tscores(sensors_.size()), qscores(sensors_.size());
        for (size_t i = 0; i < sensors_.size(); i++) {
            tscores[i] = get_d(sensors_[i].name + "_timeout_score", 1.0);
            qscores[i] = get_d(sensors_[i].name + "_quality_score", 1.0);
            scores[i] = std::clamp(tscores[i] * qscores[i], 0.0, 1.0);
        }

        // --- layer 2: 8-sector coverage from lidars (redundancy fusion) ---
        double sectors[8];
        for (int k = 0; k < 8; k++) {
            double miss = 1.0;
            for (size_t i = 0; i < sensors_.size(); i++) {
                if (sensors_[i].type != "lidar") continue;
                if (covers(sensors_[i], k * 45.0)) miss *= (1.0 - scores[i]);
            }
            sectors[k] = 1.0 - miss;
        }

        // --- layer 3: capabilities ---
        double cap_tl = 0.0;   // traffic light: front-covering cameras only
        bool has_cam = false;
        for (size_t i = 0; i < sensors_.size(); i++) {
            if (sensors_[i].type == "camera" && covers(sensors_[i], 0.0)) {
                cap_tl = std::max(cap_tl, scores[i]);
                has_cam = true;
            }
        }
        if (!has_cam) cap_tl = 0.0;

        // obstacle: forward-weighted sector mix (F,FL,FR dominate while driving)
        static const double W[8] = {0.30, 0.15, 0.10, 0.05, 0.10, 0.05, 0.10, 0.15};
        double cap_obs = 0.0;
        for (int k = 0; k < 8; k++) cap_obs += W[k] * sectors[k];

        bool imu_failed = false, gps_failed = false;
        (void)bb->get("IMU_failed", imu_failed);
        (void)bb->get("GPS_failed", gps_failed);
        const double cap_loc = (imu_failed ? 0.0 : 1.0) * (gps_failed ? 0.0 : 1.0);

        const double perception = 0.6 * cap_obs + 0.4 * cap_tl;

        // --- publish /monitor/scores ---
        diagnostic_msgs::msg::DiagnosticArray out;
        out.header.stamp = node_->now();
        for (size_t i = 0; i < sensors_.size(); i++) {
            out.status.push_back(make_status(
                "sensor/" + sensors_[i].name, scores[i],
                {{"timeout_score", tscores[i]}, {"quality_score", qscores[i]}}));
        }
        for (int k = 0; k < 8; k++)
            out.status.push_back(make_status(std::string("sector/") + SECTOR_LABELS[k],
                                             sectors[k], {}));
        out.status.push_back(make_status("capability/traffic_light", cap_tl, {}));
        out.status.push_back(make_status("capability/obstacle", cap_obs, {}));
        out.status.push_back(make_status("capability/localization", cap_loc, {}));
        out.status.push_back(make_status("capability/perception", perception, {}));
        scores_pub_->publish(out);

        // Warmup: until every configured sensor has delivered at least one
        // message, the "faults" are just the leaderboard still spawning
        // sensors — report OK to the Autoware graph so engagement is not
        // blocked or flapped, and keep the recovery latches out of it.
        if (!warmup_done_) {
            bool all_seen = true;
            for (const auto& s : sensors_) {
                bool failed = true;
                if (!bb->get(s.name + "_failed", failed)) failed = true;
                if (failed) { all_seen = false; break; }
            }
            const double waited = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - warmup_start_).count();
            if (all_seen || waited > warmup_timeout_s_) {
                warmup_done_ = true;
                cout << "[Scores] warmup done (" << waited << "s, all_seen="
                     << all_seen << ")" << endl;
            }
        }

        // --- publish /diagnostics for the Autoware aggregator ---
        // Levels are latched with hysteresis: degradation propagates instantly,
        // recovery requires score > threshold + margin held for recovery_hold_s.
        // Without this the score sawtooth (and the stop -> TTC relax -> resume
        // loop) flaps ERROR<->OK and the MRM keeps cancelling before SUCCEEDED.
        const int64_t now_ns = rclcpp::Time(out.header.stamp).nanoseconds();
        diagnostic_msgs::msg::DiagnosticArray diag;
        diag.header.stamp = out.header.stamp;
        for (size_t i = 0; i < sensors_.size(); i++) {
            auto st = make_status("cdr_monitor: sensing/ftti/" + sensors_[i].name,
                                  scores[i],
                                  {{"timeout_score", tscores[i]},
                                   {"quality_score", qscores[i]}});
            st.hardware_id = "cdr_monitor";
            if (warmup_done_) apply_latch(st, scores[i], now_ns);
            else { st.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
                   st.message = "warmup"; }
            diag.status.push_back(st);
        }
        auto agg = make_status("cdr_monitor: sensing/perception_score", perception, {});
        agg.hardware_id = "cdr_monitor";
        // The graph-linked unit must be BINARY: operation-mode availability
        // treats anything but OK as unavailable (level()==OK check), so a
        // WARN here would trigger the stock MRM in exactly the band the
        // supervisor is supposed to own. OK above the floor, ERROR below.
        agg.level = perception > 0.3 ? diagnostic_msgs::msg::DiagnosticStatus::OK
                                     : diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        if (warmup_done_) apply_latch(agg, perception, now_ns, true);
        else { agg.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
               agg.message = "warmup"; }
        diag.status.push_back(agg);
        diag_pub_->publish(diag);

        if (++tick_count_ % 10 == 1)
            cout << "[Scores] perception " << perception << " | tl " << cap_tl
                 << " obs " << cap_obs << " loc " << cap_loc << endl;
        return BT::NodeStatus::SUCCESS;
    }

private:
    static bool covers(const SensorEntry& s, double sector_center_deg)
    {
        if (s.fov_deg >= 360.0) return true;
        double d = std::fmod(std::fabs(sector_center_deg - s.yaw_deg), 360.0);
        if (d > 180.0) d = 360.0 - d;
        return d <= s.fov_deg / 2.0 + 22.5;   // sector half-width overlap
    }

    diagnostic_msgs::msg::DiagnosticStatus make_status(
        const std::string& name, double score,
        std::initializer_list<std::pair<const char*, double>> extras)
    {
        diagnostic_msgs::msg::DiagnosticStatus st;
        st.name = name;
        st.level = score > 0.7 ? diagnostic_msgs::msg::DiagnosticStatus::OK
                 : score > 0.3 ? diagnostic_msgs::msg::DiagnosticStatus::WARN
                               : diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        std::ostringstream os;
        os.precision(3);
        os << score;
        st.message = os.str();
        diagnostic_msgs::msg::KeyValue kv;
        kv.key = "score";
        kv.value = st.message;
        st.values.push_back(kv);
        for (const auto& [k, v] : extras) {
            std::ostringstream vs;
            vs.precision(3);
            vs << v;
            kv.key = k;
            kv.value = vs.str();
            st.values.push_back(kv);
        }
        return st;
    }

    struct Latch {
        uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        int64_t better_since_ns = -1;   // -1: recovery timer not running
    };

    void apply_latch(diagnostic_msgs::msg::DiagnosticStatus& st, double score,
                     int64_t now_ns, bool binary = false)
    {
        using DS = diagnostic_msgs::msg::DiagnosticStatus;
        auto& l = latches_[st.name];
        const uint8_t raw = st.level;
        if (raw >= l.level) {                    // same or worse: follow instantly
            l.level = raw;
            l.better_since_ns = -1;
        } else {
            // improvement must clear threshold + margin for recovery_hold_s
            const uint8_t rec = binary
                ? (score > 0.3 + recovery_margin_ ? DS::OK : DS::ERROR)
                : score > 0.7 + recovery_margin_ ? DS::OK
                : score > 0.3 + recovery_margin_ ? DS::WARN
                                                 : DS::ERROR;
            if (rec >= l.level) {
                l.better_since_ns = -1;          // inside the hysteresis band
            } else if (l.better_since_ns < 0) {
                l.better_since_ns = now_ns;
            } else if (now_ns - l.better_since_ns >=
                       static_cast<int64_t>(recovery_hold_s_ * 1e9)) {
                l.level = rec;
                l.better_since_ns = -1;
            }
        }
        st.level = l.level;
        if (st.level != raw) st.message += " [latched]";
    }

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr scores_pub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
    std::vector<SensorEntry> sensors_;
    std::map<std::string, Latch> latches_;
    double recovery_hold_s_ = 3.0;
    double recovery_margin_ = 0.2;
    bool warmup_done_ = false;
    double warmup_timeout_s_ = 120.0;
    std::chrono::steady_clock::time_point warmup_start_;
    uint64_t tick_count_ = 0;
};
