/**
 * [benchmark_node.cpp] (보조 도구) 메시지 수신 3가지 방식의 비용 벤치마크.
 *   같은 토픽을 세 방법(A/B/C)으로 동시에 구독해서, 콜백 안에서 걸린 시간을
 *   CSV(/tmp/cdr_benchmark.csv)로 기록한다. 논문의 "CDR 부분 파싱이 얼마나
 *   싼가" 근거 자료용. A=일반 typed 구독, B=generic 구독 + 전체 역직렬화,
 *   C=generic 구독 + 타임스탬프만 CDR 부분 파싱(우리 방식).
 */
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

        // ── 방법 A: 일반 typed 구독 ──
        // 주의: 역직렬화는 콜백이 불리기 "전에" executor가 이미 해 버린다.
        // 그래서 여기서 잰 callback_us는 스탬프 읽기 시간뿐 (실제보다 작게 나옴).
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

        // ── 방법 B: generic 구독 → 콜백 안에서 메시지 전체를 역직렬화 ──
        // 원시 바이트로 받아서 직접 풀기 때문에 역직렬화 비용이 콜백 시간에 그대로 잡힌다
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

        // ── 방법 C: generic 구독 → CDR 부분 파싱 (타임스탬프만) — 우리 방식 ──
        // 전체를 풀지 않고 헤더의 스탬프 바이트만 읽는다 → B보다 훨씬 싸다는 걸 보이는 게 목적
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
    // 측정 결과 한 건을 CSV에 기록. 세 콜백이 공용으로 부른다.
    void record(const char* method, int seq,
                hrc::time_point t0, hrc::time_point t1,
                int64_t stamp_ns, size_t bytes)
    {
        double us = std::chrono::duration<double, std::micro>(t1 - t0).count();   // 콜백 소요 시간 [us]
        int64_t recv_ns = now().nanoseconds();

        // mutex = 한 번에 한 스레드만 통과시키는 자물쇠. 여러 콜백이 동시에
        // CSV에 쓰다가 줄이 섞이는 걸 막는다. lock_guard는 이 블록을 나갈 때 자동으로 풀어줌.
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
