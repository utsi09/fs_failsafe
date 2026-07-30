/**
 * Sensor timeout recorder — fair A/B comparison for paper.
 *
 * Mode "typed":  create_subscription<T>       (normal ROS2 approach)
 * Mode "cdr":    create_generic_subscription   + CDR stamp parse
 *
 * Both use wall-clock (steady_clock) for timing → captures real
 * deserialization overhead that sim_time hides.
 *
 * Output CSV: same format as sensor_log.csv
 */
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include "hesai_zed_fusion/msg/obstacle_array.hpp"
#include "cdr_monitor/cdr_parser.hpp"

#include <chrono>
#include <fstream>
#include <mutex>
#include <cmath>
#include <atomic>

using SC = std::chrono::steady_clock;

class RecorderNode : public rclcpp::Node
{
public:
    RecorderNode() : Node("recorder_node")
    {
        declare_parameter<std::string>("mode", "cdr");          // "typed" or "cdr"
        declare_parameter<std::string>("csv_path", "/tmp/cdr_sensor_log.csv");

        mode_     = get_parameter("mode").as_string();
        csv_path_ = get_parameter("csv_path").as_string();

        csv_.open(csv_path_);
        csv_ << "time_s,speed_kmh,obstacle_m,ttc_ms,"
                "lidar_timeout_ms,lidar_ftti_ms,lidar_fault,"
                "camera_timeout_ms,camera_ftti_ms,camera_fault,"
                "imu_timeout_ms,imu_ftti_ms,imu_fault,"
                "gps_timeout_ms,gps_ftti_ms,gps_fault\n";

        rclcpp::QoS qos_be(10);
        qos_be.best_effort();

        // velocity & obstacle — always typed (small msgs, need data)
        sub_vel_ = create_subscription<geometry_msgs::msg::TwistStamped>(
            "/vel", 10,
            [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
                double vx = msg->twist.linear.x;
                double vy = msg->twist.linear.y;
                speed_ms_.store(std::sqrt(vx*vx + vy*vy) * 1e6);
            });

        // ── sensor subscriptions ──
        if (mode_ == "typed") {
            sub_lidar_t_ = create_subscription<sensor_msgs::msg::PointCloud2>(
                "/lidar_points", qos_be,
                [this](const sensor_msgs::msg::PointCloud2::SharedPtr) {
                    wall_recv_[0].store(SC::now().time_since_epoch().count());
                });
            sub_cam_t_ = create_subscription<sensor_msgs::msg::CompressedImage>(
                "/zed/zed_node/rgb/color/rect/image/compressed", qos_be,
                [this](const sensor_msgs::msg::CompressedImage::SharedPtr) {
                    wall_recv_[1].store(SC::now().time_since_epoch().count());
                });
            sub_imu_t_ = create_subscription<sensor_msgs::msg::Imu>(
                "/imu/data", qos_be,
                [this](const sensor_msgs::msg::Imu::SharedPtr) {
                    wall_recv_[2].store(SC::now().time_since_epoch().count());
                });
            sub_gps_t_ = create_subscription<sensor_msgs::msg::NavSatFix>(
                "/fix", qos_be,
                [this](const sensor_msgs::msg::NavSatFix::SharedPtr) {
                    wall_recv_[3].store(SC::now().time_since_epoch().count());
                });
        } else {
            auto make_cb = [this](int idx) {
                return [this, idx](std::shared_ptr<rclcpp::SerializedMessage> msg) {
                    (void)cdr_parser::parse_stamp_ns(msg);   // CDR parse
                    wall_recv_[idx].store(SC::now().time_since_epoch().count());
                };
            };
            sub_lidar_g_ = create_generic_subscription(
                "/lidar_points", "sensor_msgs/msg/PointCloud2",
                rclcpp::SensorDataQoS(), make_cb(0));
            sub_cam_g_ = create_generic_subscription(
                "/zed/zed_node/rgb/color/rect/image/compressed",
                "sensor_msgs/msg/CompressedImage",
                rclcpp::SensorDataQoS(), make_cb(1));
            sub_imu_g_ = create_generic_subscription(
                "/imu/data", "sensor_msgs/msg/Imu",
                rclcpp::SensorDataQoS(), make_cb(2));
            sub_gps_g_ = create_generic_subscription(
                "/fix", "sensor_msgs/msg/NavSatFix",
                rclcpp::SensorDataQoS(), make_cb(3));
        }

        timer_ = create_wall_timer(std::chrono::milliseconds(50),
                                   std::bind(&RecorderNode::tick, this));

        sub_obs_ = create_subscription<hesai_zed_fusion::msg::ObstacleArray>(
            "/hesai_zed/obstacles", 10,
            [this](const hesai_zed_fusion::msg::ObstacleArray::SharedPtr msg) {
                double min_d = 1000.0;
                for (const auto& o : msg->obstacles) {
                    if (o.distance < min_d) min_d = o.distance;
                }
                obstacle_dist_um_.store(static_cast<int64_t>(min_d * 1e6));
            });

        RCLCPP_INFO(get_logger(), "Recorder [%s] → %s",
                     mode_.c_str(), csv_path_.c_str());
    }

    ~RecorderNode()
    {
        csv_.flush();
        csv_.close();
        RCLCPP_INFO(get_logger(), "Saved %d rows → %s", rows_, csv_path_.c_str());
    }

private:
    void tick()
    {
        int64_t now_ns = SC::now().time_since_epoch().count();
        if (t0_ == 0) t0_ = now_ns;
        double t_rel = (now_ns - t0_) / 1e9;

        double v = speed_ms_.load() / 1e6;   // m/s
        double d = obstacle_dist_um_.load() / 1e6;
        if (d <= 0.0) d = 1000.0;
        double ttc = (v > 0.1) ? (d / v * 1000.0) : 99999.0;

        static const char* names[] = {"lidar", "camera", "imu", "gps"};
        static const double periods[] = {0, 0, 10.0, 50.0};  // IMU 10ms, GPS 50ms
        double ftti_default = 850.0;

        std::string row;
        char buf[64];
        snprintf(buf, sizeof(buf), "%.3f,%.2f,%.2f,%.1f",
                 t_rel, v * 3.6, d, ttc);
        row = buf;

        for (int i = 0; i < 4; i++) {
            int64_t recv = wall_recv_[i].load();
            double timeout_ms = -1.0;
            if (recv > 0)
                timeout_ms = (now_ns - recv) / 1e6;

            double ftti = std::min(ttc, ftti_default);

            int fault = 0;
            if (i < 2) {  // lidar, camera
                fault = (timeout_ms < 0 || timeout_ms > ftti) ? 1 : 0;
            } else {       // imu, gps
                if (timeout_ms < 0 || timeout_ms > periods[i] * 3)
                    fail_count_[i]++;
                else
                    fail_count_[i] = 0;
                fault = (fail_count_[i] >= 3) ? 1 : 0;
            }

            snprintf(buf, sizeof(buf), ",%.1f,%.1f,%d",
                     timeout_ms, ftti, fault);
            row += buf;
        }

        csv_ << row << "\n";
        rows_++;
        if (rows_ % 200 == 0) csv_.flush();
    }

    std::string mode_, csv_path_;
    std::ofstream csv_;
    int rows_ = 0;
    int64_t t0_ = 0;

    std::atomic<int64_t> wall_recv_[4] = {{0},{0},{0},{0}};
    std::atomic<int64_t> speed_ms_{0};   // speed * 1e6 stored as int
    std::atomic<int64_t> obstacle_dist_um_{static_cast<int64_t>(1000.0 * 1e6)};
    int fail_count_[4] = {0,0,0,0};

    // typed subs
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_t_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_cam_t_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_t_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_gps_t_;
    // generic subs
    rclcpp::GenericSubscription::SharedPtr sub_lidar_g_, sub_cam_g_,
                                           sub_imu_g_, sub_gps_g_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_vel_;
    rclcpp::Subscription<hesai_zed_fusion::msg::ObstacleArray>::SharedPtr sub_obs_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RecorderNode>());
    rclcpp::shutdown();
    return 0;
}
