/**
 * 같은 메시지를 typed sub / CDR sub 동시에 받아서
 * wall-clock 도착 시간 차이를 기록.
 *
 * single-threaded executor → CDR sub(먼저 생성) 콜백이 먼저 실행,
 * typed sub 콜백은 executor가 역직렬화 후 실행.
 * diff = wall_typed - wall_cdr ≈ 역직렬화 소요 시간.
 */
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include "cdr_monitor/cdr_parser.hpp"

#include <chrono>
#include <fstream>
#include <string>

using SC = std::chrono::steady_clock;

class LatencyCompare : public rclcpp::Node
{
public:
    LatencyCompare() : Node("latency_compare")
    {
        declare_parameter<std::string>("csv_path", "/tmp/latency_compare.csv");
        csv_path_ = get_parameter("csv_path").as_string();

        csv_.open(csv_path_);
        csv_ << "sensor,stamp_ns,wall_cdr_ns,wall_typed_ns,deser_us\n";

        auto qos = rclcpp::SensorDataQoS();

        g_lidar_ = create_generic_subscription(
            "/lidar_points", "sensor_msgs/msg/PointCloud2", qos,
            [this](std::shared_ptr<rclcpp::SerializedMessage> msg) {
                cdr_cb(0, cdr_parser::parse_stamp_ns(msg));
            });
        g_cam_ = create_generic_subscription(
            "/zed/zed_node/rgb/color/rect/image/compressed",
            "sensor_msgs/msg/CompressedImage", qos,
            [this](std::shared_ptr<rclcpp::SerializedMessage> msg) {
                cdr_cb(1, cdr_parser::parse_stamp_ns(msg));
            });
        g_imu_ = create_generic_subscription(
            "/imu/data", "sensor_msgs/msg/Imu", qos,
            [this](std::shared_ptr<rclcpp::SerializedMessage> msg) {
                cdr_cb(2, cdr_parser::parse_stamp_ns(msg));
            });
        g_gps_ = create_generic_subscription(
            "/fix", "sensor_msgs/msg/NavSatFix", qos,
            [this](std::shared_ptr<rclcpp::SerializedMessage> msg) {
                cdr_cb(3, cdr_parser::parse_stamp_ns(msg));
            });

        rclcpp::QoS qos_be(10);
        qos_be.best_effort();

        t_lidar_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "/lidar_points", qos_be,
            [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                typed_cb(0, to_ns(msg->header.stamp));
            });
        t_cam_ = create_subscription<sensor_msgs::msg::CompressedImage>(
            "/zed/zed_node/rgb/color/rect/image/compressed", qos_be,
            [this](const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
                typed_cb(1, to_ns(msg->header.stamp));
            });
        t_imu_ = create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", qos_be,
            [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
                typed_cb(2, to_ns(msg->header.stamp));
            });
        t_gps_ = create_subscription<sensor_msgs::msg::NavSatFix>(
            "/fix", qos_be,
            [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
                typed_cb(3, to_ns(msg->header.stamp));
            });

        RCLCPP_INFO(get_logger(), "Latency compare started → %s", csv_path_.c_str());
    }

    ~LatencyCompare()
    {
        csv_.flush();
        csv_.close();
        RCLCPP_INFO(get_logger(), "Saved %d rows → %s", rows_, csv_path_.c_str());
    }

private:
    static constexpr const char* NAMES[] = {"lidar", "camera", "imu", "gps"};

    int64_t to_ns(const builtin_interfaces::msg::Time& t) {
        return static_cast<int64_t>(t.sec) * 1000000000LL + t.nanosec;
    }

    void cdr_cb(int idx, int64_t stamp_ns) {
        last_cdr_wall_[idx] = SC::now().time_since_epoch().count();
        last_cdr_stamp_[idx] = stamp_ns;
    }

    void typed_cb(int idx, int64_t stamp_ns) {
        int64_t wall_typed = SC::now().time_since_epoch().count();

        if (last_cdr_stamp_[idx] == stamp_ns && last_cdr_wall_[idx] > 0) {
            double deser_us = (wall_typed - last_cdr_wall_[idx]) / 1000.0;
            csv_ << NAMES[idx] << "," << stamp_ns << ","
                 << last_cdr_wall_[idx] << "," << wall_typed << ","
                 << deser_us << "\n";
            rows_++;
            if (rows_ % 100 == 0) {
                csv_.flush();
                RCLCPP_INFO(get_logger(), "%d samples collected", rows_);
            }
        }
    }

    std::string csv_path_;
    std::ofstream csv_;
    int rows_ = 0;

    int64_t last_cdr_stamp_[4] = {0,0,0,0};
    int64_t last_cdr_wall_[4]  = {0,0,0,0};

    rclcpp::GenericSubscription::SharedPtr g_lidar_, g_cam_, g_imu_, g_gps_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr t_lidar_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr t_cam_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr t_imu_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr t_gps_;
};

constexpr const char* LatencyCompare::NAMES[];

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LatencyCompare>());
    rclcpp::shutdown();
    return 0;
}
