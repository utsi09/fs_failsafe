#pragma once
#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <mutex>
using namespace std;

// Estimates pointcloud quality (0~1) from the actual data, independent of the
// timeout/FTTI check: gaussian range noise raises the neighbor range difference,
// dropout lowers the point count. quality = noise_score * count_score.
// Additionally runs a CUSUM sequential test on the sigma excess: small but
// persistent degradations accumulate and force the score down to 0.2.
class LidarQualityChecker : public BT::SyncActionNode
{
public:
    LidarQualityChecker(const std::string& name, const BT::NodeConfig& conf,
                        const BT::RosNodeParams& params)
                        : SyncActionNode(name, conf)
    {
        node_ = params.nh.lock();
        std::string topic;
        if (getInput("topic_name", topic) && !topic.empty()) {
            sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
                topic, rclcpp::SensorDataQoS(),
                [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                    std::lock_guard<std::mutex> lk(mtx_);
                    last_msg_ = msg;
                });
        }
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("topic_name"),
            // CUSUM 파라미터: k=허용 슬랙(<0이면 baseline의 0.5배 자동), h=결정 임계 [m]
            BT::InputPort<double>("cusum_k", -1.0, "CUSUM slack k [m] (<0: 0.5*baseline auto)"),
            BT::InputPort<double>("cusum_h", 0.5, "CUSUM decision threshold h [m]"),
        };
    }

    BT::NodeStatus tick() override
    {
        sensor_msgs::msg::PointCloud2::SharedPtr msg;
        {
            std::lock_guard<std::mutex> lk(mtx_);
            msg = last_msg_;
        }
        if (!msg) {
            // 데이터 없음: 품질 추정 불가 → valid=false 별도 키로 알림
            // (점수는 기존 소비자 호환을 위해 1.0 유지, 부재 자체는 timeout 검사 담당)
            publish(1.0, false);
            return BT::NodeStatus::SUCCESS;
        }

        // 중복 스탬프 가드: 직전 처리 메시지와 같은 header.stamp이면 같은 데이터를
        // 두 번 학습(기준선/frames_ 갱신)하지 않고 이전 품질만 재출력
        const uint64_t stamp_ns =
            uint64_t(msg->header.stamp.sec) * 1000000000ull + msg->header.stamp.nanosec;
        if (has_prev_stamp_ && stamp_ns == last_stamp_ns_) {
            publish(last_quality_, last_valid_);
            return BT::NodeStatus::SUCCESS;
        }
        has_prev_stamp_ = true;
        last_stamp_ns_ = stamp_ns;

        int ox = -1, oy = -1, oz = -1;
        for (const auto& f : msg->fields) {
            if (f.datatype != sensor_msgs::msg::PointField::FLOAT32) continue;
            if (f.name == "x") ox = f.offset;
            else if (f.name == "y") oy = f.offset;
            else if (f.name == "z") oz = f.offset;
        }
        const size_t n = static_cast<size_t>(msg->width) * msg->height;

        // finite(x,y,z)인 유효 포인트만 카운트 (NaN/inf 패딩은 드롭아웃과 동일 취급)
        size_t n_valid = 0;
        if (ox >= 0 && oy >= 0 && oz >= 0 &&
            msg->data.size() >= n * msg->point_step) {
            for (size_t i = 0; i < n; i++) {
                const uint8_t* p = &msg->data[i * msg->point_step];
                float x, y, z;
                memcpy(&x, p + ox, 4); memcpy(&y, p + oy, 4); memcpy(&z, p + oz, 4);
                if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) n_valid++;
            }
        }
        if (ox < 0 || oy < 0 || oz < 0 || n_valid < 100) {
            // 스키마 불량 / 유효 포인트 부족: 품질 추정 불가 → valid=false
            publish(1.0, false);
            return BT::NodeStatus::SUCCESS;
        }

        // sigma estimate from consecutive-point range differences.
        // For i.i.d. gaussian range noise, E|r1 - r2| = 2*sigma/sqrt(pi).
        const size_t stride = std::max<size_t>(1, n / 4000);
        double acc = 0.0;
        size_t cnt = 0;
        // [v4.6] 신형 열화 통계: 근거리 클러터(강수), 원거리 결손(감쇠/오염),
        // 8-섹터 히스토그램(각도 실명) — 전수 카운트 (스트라이드 아님)
        size_t n_near = 0, n_far = 0;
        size_t sect_cnt[8] = {0, 0, 0, 0, 0, 0, 0, 0};
        size_t sect_far[8] = {0, 0, 0, 0, 0, 0, 0, 0};
        auto range_at = [&](size_t i) {
            const uint8_t* p = &msg->data[i * msg->point_step];
            float x, y, z;
            memcpy(&x, p + ox, 4); memcpy(&y, p + oy, 4); memcpy(&z, p + oz, 4);
            return std::sqrt(x * x + y * y + z * z);
        };
        for (size_t i = 0; i < n; i++) {
            const uint8_t* p = &msg->data[i * msg->point_step];
            float x, y, z;
            memcpy(&x, p + ox, 4); memcpy(&y, p + oy, 4); memcpy(&z, p + oz, 4);
            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
            const double r = std::sqrt(double(x) * x + double(y) * y + double(z) * z);
            if (r < 0.5) continue;
            if (r < 12.0) n_near++;
            int s = int(std::floor((std::atan2(y, x) + M_PI) / (M_PI / 4.0)));
            if (s < 0) s = 0;
            if (s > 7) s = 7;
            sect_cnt[s]++;
            if (r > 35.0) { n_far++; sect_far[s]++; }
        }
        for (size_t i = 0; i + 1 < n; i += stride) {
            const double r1 = range_at(i), r2 = range_at(i + 1);
            if (!std::isfinite(r1) || !std::isfinite(r2) || r1 < 0.5 || r2 < 0.5) continue;
            const double dr = std::fabs(r1 - r2);
            if (dr < 1.0) { acc += dr; cnt++; }   // reject real depth edges
        }
        const double sigma_est = cnt > 20 ? (acc / cnt) * 0.886 : 0.0;  // sqrt(pi)/2

        // slow baseline of the clean scene so structural roughness isn't
        // counted as noise; refuses to follow sudden (fault-like) jumps
        if (frames_ < 30) {
            baseline_ = frames_ == 0 ? sigma_est : 0.8 * baseline_ + 0.2 * sigma_est;
        } else if (sigma_est < baseline_ * 1.5) {
            baseline_ = 0.99 * baseline_ + 0.01 * sigma_est;
        }
        frames_++;

        const double noise_score = std::exp(-std::max(0.0, sigma_est - baseline_) / 0.1);

        if (frames_ < 30) {
            count_baseline_ = count_baseline_ == 0.0 ? double(n_valid)
                                                     : 0.8 * count_baseline_ + 0.2 * n_valid;
        } else if (n_valid > count_baseline_ * 0.95) {
            count_baseline_ = 0.99 * count_baseline_ + 0.01 * n_valid;
        }
        // [v4.6c] 카운트 샤프 커브: CARLA 라이다 카운트는 ±0.7%로 매우 안정
        // (nf 실측 46.6k~47.2k) — lso 소실을 카운트 채널로 감지한다.
        // ratio 0.99+ -> ~1.0, 0.91(lso-mid) -> ~0.49, 0.78(lso-high) -> ~0.07
        const double count_ratio =
            count_baseline_ > 0 ? std::clamp(n_valid / count_baseline_, 0.0, 1.0) : 1.0;
        const double cr = std::clamp((count_ratio - 0.7) / 0.3, 0.0, 1.0);
        const double count_score = cr * cr;

        // [v4.6] 신형 열화 점수 3종 — 베이스라인은 기존 count와 같은 정책
        // (30프레임 학습, 건강할 때만 천천히 추종, 고장형 급변은 거부)
        if (frames_ < 30) {
            near_base_ = near_base_ == 0.0 ? double(n_near)
                                           : 0.8 * near_base_ + 0.2 * n_near;
            far_base_ = far_base_ == 0.0 ? double(n_far)
                                         : 0.8 * far_base_ + 0.2 * n_far;
            for (int s = 0; s < 8; s++) {
                sect_base_[s] = sect_base_[s] == 0.0
                    ? double(sect_cnt[s]) : 0.8 * sect_base_[s] + 0.2 * sect_cnt[s];
                sect_far_base_[s] = sect_far_base_[s] == 0.0
                    ? double(sect_far[s]) : 0.8 * sect_far_base_[s] + 0.2 * sect_far[s];
            }
        } else {
            // [v4.6c] 추종 가드 0.95x/1.1x: 고장 수준 변화를 천천히 흡수해
            // 감지가 무뎌지던 버그 수정 (0.7x는 10 s 만에 고장값 수렴, 실측)
            if (n_near < near_base_ * 1.1)
                near_base_ = 0.99 * near_base_ + 0.01 * n_near;
            if (n_far > far_base_ * 0.9)
                far_base_ = 0.99 * far_base_ + 0.01 * n_far;
            for (int s = 0; s < 8; s++) {
                if (sect_cnt[s] > sect_base_[s] * 0.9)
                    sect_base_[s] = 0.99 * sect_base_[s] + 0.01 * sect_cnt[s];
                if (sect_far[s] > sect_far_base_[s] * 0.9)
                    sect_far_base_[s] = 0.99 * sect_far_base_[s] + 0.01 * sect_far[s];
            }
        }

        // 근거리 클러터(강수): r<12 m 포인트가 베이스라인 1.3배 초과분만큼 감점
        const double clutter_excess = near_base_ > 50
            ? std::max(0.0, n_near / near_base_ - 1.3) : 0.0;
        const double clutter_score = std::exp(-clutter_excess / 0.5);
        // 원거리 결손(감쇠): r>35 m 포인트 잔존율. 0.7x 슬랙 — 장면 변화
        // (개활지->시가지)의 자연 변동에 오탐하지 않게 30% 여유
        const double far_score = far_base_ > 200
            ? std::clamp(n_far / (0.7 * far_base_), 0.0, 1.0) : 1.0;
        // 섹터 결손(각도 실명): 최악 섹터 잔존율. 총카운트는 근거리 지면점이
        // 지배해 섹터 거리붕괴를 못 봐서(v4.6b 실측 worst 0.97), 방향별
        // "원거리(r>35m) 카운트" 잔존율로 판정 — 섹터 range collapse 직접 측정.
        // 총카운트 판정은 완전 실명(섹터 전멸)용 보조로 유지.
        double sector_score = 1.0;
        for (int s = 0; s < 8; s++) {
            if (sect_far_base_[s] > 150)
                sector_score = std::min(
                    sector_score,
                    std::clamp(sect_far[s] / (0.7 * sect_far_base_[s]), 0.0, 1.0));
            if (sect_base_[s] > 300)
                sector_score = std::min(
                    sector_score,
                    std::clamp(sect_cnt[s] / (0.7 * sect_base_[s]), 0.0, 1.0));
        }

        // [v4.6c] far/섹터 감지기는 장면 의존(개활지·울타리 오탐)이라 quality에서
        // 제외 — 로그 관측용으로만 유지. lso는 카운트 샤프커브, lcl은 클러터+
        // 시그마 채널이 담당 (둘 다 장면 강건 실측)
        double quality = std::clamp(noise_score * count_score, 0.0, 1.0);
        quality = std::min(quality, clutter_score);
        (void)far_score;
        (void)sector_score;

        // CUSUM 순차검정: exp() 점수와 병행. sigma의 baseline 초과분에서 슬랙 k를
        // 뺀 값을 S=max(0,S+excess-k)로 누적 — 작은 고장도 시간이 지나면 h를
        // 넘으므로 감지 지연이 고장 크기에 반비례한다. h 초과 = 강한 증거로 보고
        // 품질을 즉시 0.2로 강하. (워밍업 30프레임 이후에만 작동)
        if (frames_ > 30) {
            double k = -1.0, h = 0.5;
            getInput("cusum_k", k);
            getInput("cusum_h", h);
            if (k < 0.0) k = 0.5 * baseline_;   // 기본 슬랙: baseline의 0.5배
            const double excess = sigma_est - baseline_;
            cusum_s_ = std::max(0.0, cusum_s_ + excess - k);
            if (cusum_s_ > h) quality = std::min(quality, 0.2);
        }

        publish(quality, true);
        cout << name() << " quality: " << quality << " (sigma " << sigma_est
             << "m, base " << baseline_ << "m, pts " << n_valid << "/" << n
             << ", cusum " << cusum_s_
             << ", near " << n_near << "/" << int(near_base_)
             << ", far " << n_far << "/" << int(far_base_)
             << ", worst_sect " << sector_score << ")" << endl;
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
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    sensor_msgs::msg::PointCloud2::SharedPtr last_msg_;
    std::mutex mtx_;
    double baseline_ = 0.0;
    double count_baseline_ = 0.0;
    double near_base_ = 0.0;
    double far_base_ = 0.0;
    double sect_base_[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    double sect_far_base_[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    int frames_ = 0;
    uint64_t last_stamp_ns_ = 0;
    bool has_prev_stamp_ = false;
    double last_quality_ = 1.0;
    bool last_valid_ = false;
    double cusum_s_ = 0.0;
};
