#pragma once
#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include "cdr_monitor/cdr_parser.hpp"
#include "cdr_monitor/latency_logger.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <atomic>
#include <deque>
#include <mutex>
#include <vector>
using namespace std;

class CamChecker : public BT::SyncActionNode
{
public:
    CamChecker(const std::string& name, const BT::NodeConfig& conf,
               const BT::RosNodeParams& params)
               : SyncActionNode(name, conf)
    {
        node_ = params.nh.lock();
        std::string topic;
        std::string msg_type = "sensor_msgs/msg/CompressedImage";
        getInput("msg_type", msg_type);
        if (msg_type.empty()) msg_type = "sensor_msgs/msg/CompressedImage";
        if (getInput("topic_name", topic) && !topic.empty()) {
            topic_ = topic;
            sub_ = node_->create_generic_subscription(
                topic_, msg_type,
                rclcpp::SensorDataQoS(),
                [this](std::shared_ptr<rclcpp::SerializedMessage> msg) {
                    last_stamp_ns_.store(cdr_parser::parse_stamp_ns(msg), std::memory_order_relaxed);
                    const int64_t now_ns = node_->now().nanoseconds();
                    const int64_t prev_ns = last_recv_ns_.exchange(now_ns, std::memory_order_relaxed);
                    if (prev_ns > 0 && now_ns > prev_ns) {
                        // 공칭 주기 추정용 수신 간격 기록 (이동중앙값 계산에 사용)
                        std::lock_guard<std::mutex> lk(interval_mtx_);
                        intervals_.push_back((now_ns - prev_ns) / 1e6);
                        if (intervals_.size() > 21) intervals_.pop_front();
                    }
                    wall_recv_ns_.store(std::chrono::steady_clock::now().time_since_epoch().count(), std::memory_order_relaxed);
                }
            );
        }
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("topic_name"),
            BT::InputPort<std::string>("msg_type", "sensor_msgs/msg/CompressedImage", "ROS message type of the topic"),
            BT::InputPort<double>("cam_ftti"),
        };
    }

    BT::NodeStatus tick() override
    {
        if (topic_.empty()) {
            return BT::NodeStatus::SUCCESS;
        }
        double ftti = 850.0;
        getInput("cam_ftti", ftti);

        int64_t recv_ns = last_recv_ns_.load(std::memory_order_relaxed);
        double timeout = -1.0;
        if (recv_ns > 0) {
            timeout = (node_->now().nanoseconds() - recv_ns) / 1e6;
        }

        bool failed = (timeout < 0 || timeout > ftti);
        config().blackboard->set(name() + "_failed", failed);
        // continuous FTTI margin: 1.0 fresh, 0.0 at/after the FTTI deadline
        double timeout_score = (timeout < 0 || ftti <= 0.0)
            ? 0.0 : std::clamp(1.0 - timeout / ftti, 0.0, 1.0);
        // k-미스 침묵 감지: 공칭 주기(최근 수신 간격의 이동중앙값)의 3배 동안
        // 무소식이면 FTTI 마감 전이라도 "의심" 단계로 조기 페널티(0.5 곱)
        const double nominal_ms = nominal_period_ms();
        if (nominal_ms > 0.0 && timeout > 3.0 * nominal_ms)
            timeout_score *= 0.5;
        config().blackboard->set(name() + "_timeout_score", timeout_score);
        cout << name() << " timeout: " << timeout << "ms / ftti: " << ftti << "ms" << endl;
        LatencyLogger::instance().log(name(), last_stamp_ns_.load(), wall_recv_ns_.load());
        return failed ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
    }

private:
    // 최근 수신 간격의 이동중앙값으로 센서 공칭 주기[ms]를 추정 (표본 부족 시 0)
    double nominal_period_ms()
    {
        std::lock_guard<std::mutex> lk(interval_mtx_);
        if (intervals_.size() < 5) return 0.0;
        std::vector<double> v(intervals_.begin(), intervals_.end());
        std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
        return v[v.size() / 2];
    }

    rclcpp::Node::SharedPtr node_;
    rclcpp::GenericSubscription::SharedPtr sub_;
    std::atomic<int64_t> last_stamp_ns_{0};
    std::atomic<int64_t> last_recv_ns_{0};
    std::atomic<int64_t> wall_recv_ns_{0};
    std::mutex interval_mtx_;
    std::deque<double> intervals_;
    std::string topic_;
};
