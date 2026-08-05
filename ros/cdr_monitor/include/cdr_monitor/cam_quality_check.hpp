#pragma once
#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <algorithm>
#include <cmath>
#include <mutex>
#include <vector>
using namespace std;

// Estimates image quality (0~1) from the actual pixels: gaussian pixel noise
// raises the horizontal neighbor difference, a frozen image makes consecutive
// frames identical. quality = freeze_score * noise_score.
// Only raw sensor_msgs/Image is analyzed (compressed inputs return 1.0).
// Additionally runs a CUSUM sequential test on the noise excess: small but
// persistent degradations accumulate and force the score down to 0.2.
class CamQualityChecker : public BT::SyncActionNode
{
public:
    CamQualityChecker(const std::string& name, const BT::NodeConfig& conf,
                      const BT::RosNodeParams& params)
                      : SyncActionNode(name, conf)
    {
        node_ = params.nh.lock();
        std::string topic, msg_type;
        getInput("msg_type", msg_type);
        if (getInput("topic_name", topic) && !topic.empty() &&
            msg_type == "sensor_msgs/msg/Image") {
            sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
                topic, rclcpp::SensorDataQoS(),
                [this](sensor_msgs::msg::Image::SharedPtr msg) {
                    std::lock_guard<std::mutex> lk(mtx_);
                    last_msg_ = msg;
                });
        }
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("topic_name"),
            BT::InputPort<std::string>("msg_type", "sensor_msgs/msg/CompressedImage", "ROS message type of the topic"),
            BT::InputPort<double>("vel_x", 0.0, "ego velocity for standstill gating"),
            BT::InputPort<double>("vel_y", 0.0, "ego velocity for standstill gating"),
            // CUSUM 파라미터: k=허용 슬랙(<0이면 baseline의 0.5배 자동), h=결정 임계 [intensity]
            BT::InputPort<double>("cusum_k", -1.0, "CUSUM slack k (<0: 0.5*baseline auto)"),
            BT::InputPort<double>("cusum_h", 30.0, "CUSUM decision threshold h"),
        };
    }

    BT::NodeStatus tick() override
    {
        sensor_msgs::msg::Image::SharedPtr msg;
        {
            std::lock_guard<std::mutex> lk(mtx_);
            msg = last_msg_;
        }
        if (!msg || msg->width < 8 || msg->height < 8) {
            // 데이터 없음/스키마 불량: 품질 추정 불가 → valid=false 별도 키로 알림
            // (점수는 기존 소비자 호환을 위해 1.0 유지, 부재 자체는 timeout 검사 담당)
            publish(1.0, false);
            return BT::NodeStatus::SUCCESS;
        }

        // 중복 스탬프 가드: 직전 처리 메시지와 같은 header.stamp이면 같은 프레임을
        // 두 번 학습(기준선/frames_/freeze 카운터 갱신)하지 않고 이전 품질만 재출력.
        // BT 틱이 카메라 주기보다 빠를 때 생기는 가짜 freeze 판정도 함께 막는다.
        const uint64_t stamp_ns =
            uint64_t(msg->header.stamp.sec) * 1000000000ull + msg->header.stamp.nanosec;
        if (has_prev_stamp_ && stamp_ns == last_stamp_ns_) {
            publish(last_quality_, last_valid_);
            return BT::NodeStatus::SUCCESS;
        }
        has_prev_stamp_ = true;
        last_stamp_ns_ = stamp_ns;

        const size_t pixel_stride = msg->step / msg->width;   // 4 for bgra8
        const size_t grid = 64;
        const size_t sx = std::max<size_t>(1, msg->width / grid);
        const size_t sy = std::max<size_t>(1, msg->height / grid);

        std::vector<uint8_t> sample;
        sample.reserve(grid * grid);
        double noise_acc = 0.0;
        size_t noise_cnt = 0;
        for (size_t y = 0; y < msg->height; y += sy) {
            const uint8_t* row = &msg->data[y * msg->step];
            for (size_t x = 0; x + 1 < msg->width; x += sx) {
                const uint8_t v = row[x * pixel_stride];           // one channel is enough
                sample.push_back(v);
                noise_acc += std::fabs(double(v) - double(row[(x + 1) * pixel_stride]));
                noise_cnt++;
            }
        }
        const double noise_est = noise_cnt ? noise_acc / noise_cnt : 0.0;

        // freeze detection: identical consecutive frames for several ticks.
        // Gated by ego speed: a stopped vehicle in a static scene produces
        // identical frames legitimately (e.g. waiting at a red light).
        double vx = 0.0, vy = 0.0;
        getInput("vel_x", vx);
        getInput("vel_y", vy);
        const bool moving = std::hypot(vx, vy) >= 0.5;   // m/s
        double freeze_score = 1.0;
        if (prev_sample_.size() == sample.size()) {
            double diff = 0.0;
            for (size_t i = 0; i < sample.size(); i++)
                diff += std::fabs(double(sample[i]) - double(prev_sample_[i]));
            diff /= sample.size();
            if (moving) frozen_ticks_ = diff < 0.3 ? frozen_ticks_ + 1 : 0;
            else if (diff >= 0.3) frozen_ticks_ = 0;     // frames differ -> clearly not frozen
            if (frozen_ticks_ >= 5) freeze_score = 0.0;
        }
        prev_sample_ = std::move(sample);

        // baseline of the clean scene's natural texture
        if (frames_ < 30) {
            baseline_ = frames_ == 0 ? noise_est : 0.8 * baseline_ + 0.2 * noise_est;
        } else if (noise_est < baseline_ * 1.5) {
            baseline_ = 0.99 * baseline_ + 0.01 * noise_est;
        }
        frames_++;

        const double noise_score = std::exp(-std::max(0.0, noise_est - baseline_) / 10.0);
        double quality = std::clamp(freeze_score * noise_score, 0.0, 1.0);

        // CUSUM 순차검정: exp() 점수와 병행. noise의 baseline 초과분에서 슬랙 k를
        // 뺀 값을 S=max(0,S+excess-k)로 누적 — 작은 고장도 시간이 지나면 h를
        // 넘으므로 감지 지연이 고장 크기에 반비례한다. h 초과 = 강한 증거로 보고
        // 품질을 즉시 0.2로 강하. (워밍업 30프레임 이후에만 작동)
        if (frames_ > 30) {
            double k = -1.0, h = 30.0;
            getInput("cusum_k", k);
            getInput("cusum_h", h);
            if (k < 0.0) k = 0.5 * baseline_;   // 기본 슬랙: baseline의 0.5배
            const double excess = noise_est - baseline_;
            cusum_s_ = std::max(0.0, cusum_s_ + excess - k);
            // [v2 비활성] 카메라 화소차분은 주행 중 장면 이동(optical flow)로
            // 자연 상승(정지 기준선 3.8 -> 주행 6.4 실측)해 CUSUM이 구조적으로
            // 오적산됨(nf에서 S=1257 도달, 무고장 DEFER 유발). 속도 종속
            // 신호에는 누적 검정 부적합 — 기하 기반이라 속도 무관인 라이다
            // CUSUM만 유지한다. (cusum_s_ 값은 기록용으로 계속 계산)
            // if (cusum_s_ > h) quality = std::min(quality, 0.2);
            (void)h;
        }

        publish(quality, true);
        cout << name() << " quality: " << quality << " (noise " << noise_est
             << ", base " << baseline_ << ", frozen " << frozen_ticks_
             << ", cusum " << cusum_s_ << ")" << endl;
        return BT::NodeStatus::SUCCESS;
    }

private:
    // 점수와 valid 플래그를 blackboard에 기록하고 재출력용으로 보관
    void publish(double quality, bool valid)
    {
        config().blackboard->set(name() + "_quality_score", quality);
        config().blackboard->set(name() + "_quality_valid", valid);
        last_quality_ = quality;
        last_valid_ = valid;
    }

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    sensor_msgs::msg::Image::SharedPtr last_msg_;
    std::mutex mtx_;
    std::vector<uint8_t> prev_sample_;
    int frozen_ticks_ = 0;
    double baseline_ = 0.0;
    int frames_ = 0;
    uint64_t last_stamp_ns_ = 0;
    bool has_prev_stamp_ = false;
    double last_quality_ = 1.0;
    bool last_valid_ = false;
    double cusum_s_ = 0.0;
};
