#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "cdr_monitor/cdr_parser.hpp"
#include <fstream>
#include <chrono>
#include <mutex>

using hrc = std::chrono::high_resolution_clock;

class BenchmarkNode : public rclcpp::Node
{
public:
    BenchmarkNode() : Node("benchmark_node")
    {
        declare_parameter<std::string>("topic",
            "/zed/zed_node/rgb/color/rect/image/compressed");
        declare_parameter<std::string>("msg_type",
            "sensor_msgs/msg/CompressedImage");
        declare_parameter<std::string>("csv_path",
            "/tmp/cdr_benchmark.csv");
        declare_parameter<int>("max_samples", 1000);

        topic_     = get_parameter("topic").as_string();
        msg_type_  = get_parameter("msg_type").as_string();
        csv_path_  = get_parameter("csv_path").as_string();
        max_samples_ = get_parameter("max_samples").as_int();

        csv_.open(csv_path_);
        csv_ << "seq,method,callback_us,stamp_ns,recv_ns,msg_bytes\n";

        // ── Method A: normal typed subscription ──
        // Note: deserialization happens in the executor BEFORE callback.
        // callback_us here is only the stamp-read time (artificially low).
        if (msg_type_ == "sensor_msgs/msg/CompressedImage") {
            sub_a_img_ = create_subscription<sensor_msgs::msg::CompressedImage>(
                topic_, rclcpp::SensorDataQoS(),
                [this](const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
                    auto t0 = hrc::now();
                    int64_t stamp_ns =
                        static_cast<int64_t>(msg->header.stamp.sec) * 1000000000LL
                        + msg->header.stamp.nanosec;
                    auto t1 = hrc::now();
                    record("A", seq_a_++, t0, t1, stamp_ns, 0);
                });
        } else {
            sub_a_pc_ = create_subscription<sensor_msgs::msg::PointCloud2>(
                topic_, rclcpp::SensorDataQoS(),
                [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                    auto t0 = hrc::now();
                    int64_t stamp_ns =
                        static_cast<int64_t>(msg->header.stamp.sec) * 1000000000LL
                        + msg->header.stamp.nanosec;
                    auto t1 = hrc::now();
                    record("A", seq_a_++, t0, t1, stamp_ns, 0);
                });
        }

        // ── Method B: generic sub → full deserialization in callback ──
        sub_b_ = create_generic_subscription(
            topic_, msg_type_, rclcpp::SensorDataQoS(),
            [this](std::shared_ptr<rclcpp::SerializedMessage> msg) {
                auto t0 = hrc::now();
                size_t sz = msg->get_rcl_serialized_message().buffer_length;
                int64_t stamp_ns = 0;

                if (msg_type_ == "sensor_msgs/msg/CompressedImage") {
                    sensor_msgs::msg::CompressedImage d;
                    rclcpp::Serialization<sensor_msgs::msg::CompressedImage> ser;
                    ser.deserialize_message(msg.get(), &d);
                    stamp_ns = static_cast<int64_t>(d.header.stamp.sec) * 1000000000LL
                             + d.header.stamp.nanosec;
                } else {
                    sensor_msgs::msg::PointCloud2 d;
                    rclcpp::Serialization<sensor_msgs::msg::PointCloud2> ser;
                    ser.deserialize_message(msg.get(), &d);
                    stamp_ns = static_cast<int64_t>(d.header.stamp.sec) * 1000000000LL
                             + d.header.stamp.nanosec;
                }

                auto t1 = hrc::now();
                record("B", seq_b_++, t0, t1, stamp_ns, sz);
            });

        // ── Method C: generic sub → CDR partial parse (stamp only) ──
        sub_c_ = create_generic_subscription(
            topic_, msg_type_, rclcpp::SensorDataQoS(),
            [this](std::shared_ptr<rclcpp::SerializedMessage> msg) {
                auto t0 = hrc::now();
                int64_t stamp_ns = cdr_parser::parse_stamp_ns(msg);
                size_t sz = msg->get_rcl_serialized_message().buffer_length;
                auto t1 = hrc::now();
                record("C", seq_c_++, t0, t1, stamp_ns, sz);
            });

        RCLCPP_INFO(get_logger(),
            "Benchmark started: topic=%s  type=%s  csv=%s  max=%d",
            topic_.c_str(), msg_type_.c_str(), csv_path_.c_str(), max_samples_);
    }

    ~BenchmarkNode()
    {
        csv_.flush();
        csv_.close();
        RCLCPP_INFO(get_logger(), "Done. A=%d  B=%d  C=%d  samples → %s",
                     seq_a_, seq_b_, seq_c_, csv_path_.c_str());
    }

private:
    void record(const char* method, int seq,
                hrc::time_point t0, hrc::time_point t1,
                int64_t stamp_ns, size_t bytes)
    {
        double us = std::chrono::duration<double, std::micro>(t1 - t0).count();
        int64_t recv_ns = now().nanoseconds();

        std::lock_guard<std::mutex> lk(mu_);
        csv_ << seq << "," << method << "," << us << ","
             << stamp_ns << "," << recv_ns << "," << bytes << "\n";

        if ((seq_a_ + seq_b_ + seq_c_) % 300 == 0)
            csv_.flush();

        if (seq_a_ >= max_samples_ &&
            seq_b_ >= max_samples_ &&
            seq_c_ >= max_samples_)
        {
            csv_.flush();
            RCLCPP_INFO(get_logger(),
                "Reached %d samples per method. Shutting down.", max_samples_);
            rclcpp::shutdown();
        }
    }

    std::string topic_, msg_type_, csv_path_;
    int max_samples_;

    // A
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_a_img_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_a_pc_;
    // B, C
    rclcpp::GenericSubscription::SharedPtr sub_b_, sub_c_;

    std::ofstream csv_;
    std::mutex mu_;
    int seq_a_ = 0, seq_b_ = 0, seq_c_ = 0;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BenchmarkNode>());
    rclcpp::shutdown();
    return 0;
}
