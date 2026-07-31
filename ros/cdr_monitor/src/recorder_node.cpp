/**
 * [recorder_node.cpp] (보조 도구) 논문용 A/B 비교 실험 기록기.
 *   센서 수신 타임아웃/FTTI/고장 판정을 50ms마다 CSV 한 줄로 기록한다.
 *   mode="typed": 일반 구독(create_subscription, 메시지를 완전히 해석해서 받음)
 *   mode="cdr"  : generic 구독(원시 바이트로 받아 타임스탬프만 부분 파싱) — 더 가벼움
 *   두 방식 모두 벽시계(steady_clock)로 재서, 시뮬레이션 시간이 숨기는
 *   실제 역직렬화 비용까지 측정한다. 출력 CSV는 sensor_log.csv와 같은 형식.
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

        // 속도/장애물은 메시지가 작고 내용이 필요해서 항상 typed 구독.
        // [this](...){...} 는 람다 = "이 객체(this)를 기억하는 즉석 함수".
        // 메시지가 도착할 때마다 이 함수(콜백)가 호출된다.
        sub_vel_ = create_subscription<geometry_msgs::msg::TwistStamped>(
            "/vel", 10,
            [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
                double vx = msg->twist.linear.x;
                double vy = msg->twist.linear.y;
                speed_ms_.store(std::sqrt(vx*vx + vy*vy) * 1e6);
            });

        // ── 센서 구독 (모드에 따라 typed / cdr 중 한 방식으로) ──
        // 콜백은 수신 순간의 벽시계 시각만 wall_recv_[i]에 저장한다 (원자 변수라 스레드 안전)
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
            // cdr 모드: "콜백을 만들어 주는 람다" — idx만 바꿔 4개 센서에 재사용
            auto make_cb = [this](int idx) {
                return [this, idx](std::shared_ptr<rclcpp::SerializedMessage> msg) {
                    (void)cdr_parser::parse_stamp_ns(msg);   // 원시 바이트에서 타임스탬프만 CDR 파싱
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
    // 50ms마다 타이머가 호출: 각 센서의 "마지막 수신 후 경과 시간"과
    // FTTI(= min(TTC, 850ms)) 를 비교해 고장 여부를 판정하고 CSV 한 줄 기록
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
            if (i < 2) {  // 라이다/카메라: FTTI(동적 데드라인) 초과면 즉시 고장
                fault = (timeout_ms < 0 || timeout_ms > ftti) ? 1 : 0;
            } else {       // IMU/GPS: 주기의 3배 초과가 3번 연속이면 고장 (디바운스)
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
