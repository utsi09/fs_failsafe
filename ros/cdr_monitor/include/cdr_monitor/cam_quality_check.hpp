// ============================================================================
// [이 파일의 역할]
// 전체 시스템 흐름: 센서 고장 감시(cdr_monitor의 행동트리) → 센서별 점수 계산
//   → PublishScores가 점수를 합산해 /monitor/scores, /diagnostics로 발행
//   → MRM 슈퍼바이저가 그 점수를 보고 비상정지/우회 같은 MRM을 판단.
// 이 파일은 그중 "카메라 영상의 품질"을 0~1 점수로 매기는 행동트리 노드다.
// 데이터 입력: 카메라 원본 영상 토픽(sensor_msgs/Image)을 구독.
// 데이터 출력: 블랙보드(트리 공유 저장소)에 "<노드이름>_quality_score" 키로 저장
//   → PublishScores 노드가 이 값을 읽어감.
// 타임아웃(FTTI) 검사는 cam_check.hpp가 따로 담당하고, 여기는 "영상이 오긴
// 오는데 내용이 이상한가"(픽셀 노이즈, 화면 정지)만 본다.
// ============================================================================
#pragma once                                   // 헤더가 여러 번 include돼도 한 번만 처리
#include "behaviortree_ros2/bt_topic_sub_node.hpp"   // BehaviorTree.CPP + ROS2 연동 기반 클래스들
#include <sensor_msgs/msg/image.hpp>           // 카메라 원본 영상 메시지 타입
#include <algorithm>                           // std::max, std::clamp 등
#include <cmath>                               // exp, fabs, hypot 등
#include <mutex>                               // 뮤텍스(스레드 자물쇠)
#include <vector>                              // 동적 배열
using namespace std;                           // std:: 접두사 생략

// [원리 요약] 영상 품질을 실제 픽셀로 추정한다 (0~1):
//  - 가우시안 픽셀 노이즈가 생기면 → 가로 이웃 픽셀 간 밝기 차이가 커짐 (noise_score 하락)
//  - 영상이 얼어붙으면(freeze) → 연속 프레임이 완전히 똑같아짐 (freeze_score = 0)
//  - 최종 품질 = freeze_score * noise_score.
//  - 원본(raw) sensor_msgs/Image만 분석 가능. 압축(compressed) 입력은 그냥 1.0 반환.
// SyncActionNode = "행동트리가 주기적으로 '똑딱(tick)' 부르면 한 번 실행되고
// 즉시 결과를 돌려주는 작업 단위"
class CamQualityChecker : public BT::SyncActionNode
{
public:
    // 생성자: 트리 구성 시 한 번 호출. name=XML상의 노드 이름, conf=트리 설정(블랙보드 포함),
    // params=ROS 노드 핸들 묶음
    CamQualityChecker(const std::string& name, const BT::NodeConfig& conf,
                      const BT::RosNodeParams& params)
                      : SyncActionNode(name, conf)   // 부모 클래스(BT 노드) 초기화
    {
        // weak_ptr(약한 참조)를 shared_ptr(진짜 소유권)로 승격. 노드가 죽었으면 nullptr
        node_ = params.nh.lock();
        std::string topic, msg_type;           // 토픽 이름과 메시지 타입 문자열
        getInput("msg_type", msg_type);        // XML 입력 구멍에서 메시지 타입을 읽음
        // 토픽 이름이 있고, 타입이 "원본 Image"일 때만 구독 시작
        // (압축 영상은 픽셀 분석이 안 되므로 구독 자체를 안 함 → 점수는 항상 1.0)
        if (getInput("topic_name", topic) && !topic.empty() &&
            msg_type == "sensor_msgs/msg/Image") {
            // create_subscription = "이 토픽(방송 채널)에 새 데이터가 올 때마다
            // 람다(즉석 함수)를 실행해달라"고 ROS에 등록
            sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
                topic, rclcpp::SensorDataQoS(),   // 센서용 통신 설정(최신 우선, 유실 허용)
                [this](sensor_msgs::msg::Image::SharedPtr msg) {   // [this] = 이 객체를 캡처한 람다
                    // lock_guard = 뮤텍스 자물쇠. 블록 끝나면 자동 해제.
                    // 콜백 스레드와 tick() 스레드의 동시 접근 방지
                    std::lock_guard<std::mutex> lk(mtx_);
                    last_msg_ = msg;           // 최신 영상을 저장해 둠
                });
        }
    }

    // providedPorts = 트리 XML에서 이 노드로 값을 전달하는 "입력 구멍" 목록 선언
    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("topic_name"),   // 구독할 카메라 토픽 이름
            // (구멍이름, 기본값, 설명) 형식. 기본은 압축 영상 타입
            BT::InputPort<std::string>("msg_type", "sensor_msgs/msg/CompressedImage", "ROS message type of the topic"),
            BT::InputPort<double>("vel_x", 0.0, "ego velocity for standstill gating"),   // 자차 속도 x (정차 판정용)
            BT::InputPort<double>("vel_y", 0.0, "ego velocity for standstill gating"),   // 자차 속도 y (정차 판정용)
        };
    }

    // tick() = 트리가 주기적으로 부르는 함수. 매 호출마다 최신 프레임의 품질을 계산
    BT::NodeStatus tick() override
    {
        sensor_msgs::msg::Image::SharedPtr msg;   // 분석할 프레임
        {
            // 자물쇠 잠그고 최신 프레임 포인터만 복사 (잠금 시간 최소화)
            std::lock_guard<std::mutex> lk(mtx_);
            msg = last_msg_;
        }
        if (!msg || msg->width < 8 || msg->height < 8) {
            // 데이터가 없거나 너무 작으면 분석 불가 → 만점 처리.
            // (데이터 없음 자체는 타임아웃 검사가 담당)
            // blackboard = 트리 노드들이 공유하는 칠판(키-값 저장소)
            config().blackboard->set(name() + "_quality_score", 1.0);
            return BT::NodeStatus::SUCCESS;
        }

        // step = 한 줄(row)이 차지하는 바이트 수. 이를 폭으로 나누면 픽셀 1개의 바이트 수
        const size_t pixel_stride = msg->step / msg->width;   // bgra8 포맷이면 4바이트
        // 영상 전체를 다 보지 않고 64x64 격자로 듬성듬성 표본만 뽑는다 (계산량 절약)
        const size_t grid = 64;
        const size_t sx = std::max<size_t>(1, msg->width / grid);    // 가로 표본 간격
        const size_t sy = std::max<size_t>(1, msg->height / grid);   // 세로 표본 간격

        std::vector<uint8_t> sample;           // 이번 프레임의 표본 픽셀들 (freeze 비교용)
        sample.reserve(grid * grid);           // 메모리를 미리 확보 (재할당 방지 최적화)
        double noise_acc = 0.0;                // 이웃 픽셀 밝기차의 누적 합
        size_t noise_cnt = 0;                  // 표본 쌍 개수
        for (size_t y = 0; y < msg->height; y += sy) {          // 세로로 sy칸씩 건너뛰며
            const uint8_t* row = &msg->data[y * msg->step];     // y번째 줄의 시작 주소
            for (size_t x = 0; x + 1 < msg->width; x += sx) {   // 가로로 sx칸씩 건너뛰며
                const uint8_t v = row[x * pixel_stride];           // 픽셀의 첫 채널(예: B) 하나면 노이즈 감지에 충분
                sample.push_back(v);           // freeze 비교용 표본으로 저장
                // 바로 오른쪽 이웃 픽셀과의 밝기 차이를 누적 (노이즈가 크면 이 값이 커짐)
                noise_acc += std::fabs(double(v) - double(row[(x + 1) * pixel_stride]));
                noise_cnt++;
            }
        }
        // 평균 이웃 밝기차 = 노이즈 수준의 추정치 (표본이 없으면 0)
        const double noise_est = noise_cnt ? noise_acc / noise_cnt : 0.0;

        // [freeze(정지 화면) 감지] 연속 프레임이 여러 tick 동안 완전히 똑같으면 고장.
        // 단, 자차 속도로 게이팅: 정차 중(예: 빨간불 대기)에는 장면이 안 변해서
        // 똑같은 프레임이 정상적으로 나올 수 있으므로, 움직일 때만 freeze 카운트를 올림
        double vx = 0.0, vy = 0.0;
        getInput("vel_x", vx);                 // XML 입력 구멍에서 자차 속도를 읽음
        getInput("vel_y", vy);
        // hypot(vx,vy) = sqrt(vx^2+vy^2) = 속도 크기. 0.5 m/s 이상이면 "주행 중"으로 판정
        const bool moving = std::hypot(vx, vy) >= 0.5;   // m/s
        double freeze_score = 1.0;             // 기본은 정지 아님 → 만점
        if (prev_sample_.size() == sample.size()) {   // 이전 프레임 표본과 크기가 같아야 비교 가능
            double diff = 0.0;
            for (size_t i = 0; i < sample.size(); i++)   // 표본 픽셀별 밝기차를 전부 합산
                diff += std::fabs(double(sample[i]) - double(prev_sample_[i]));
            diff /= sample.size();             // 픽셀당 평균 차이
            // 주행 중인데 프레임이 거의 안 변했으면(평균차 < 0.3) 정지 의심 카운트 +1, 변했으면 리셋
            if (moving) frozen_ticks_ = diff < 0.3 ? frozen_ticks_ + 1 : 0;
            else if (diff >= 0.3) frozen_ticks_ = 0;     // 정차 중이라도 프레임이 변하면 확실히 정지 아님 → 리셋
            if (frozen_ticks_ >= 5) freeze_score = 0.0;  // 5tick 연속 동일 프레임 → 화면 정지 확정, 0점
        }
        // 이번 표본을 다음 tick의 "이전 프레임"으로 넘김.
        // std::move = 복사 대신 내용물을 통째로 이사시킴 (빠름, sample은 빈 상태가 됨)
        prev_sample_ = std::move(sample);

        // [베이스라인 학습] 장면이 원래 갖고 있는 자연 질감(텍스처)을 노이즈로
        // 오해하지 않도록, "깨끗할 때의 이웃차 평균"을 기준선으로 유지
        if (frames_ < 30) {
            // 처음 30프레임: 빠르게 학습 (지수이동평균, 새 값 20% 반영)
            baseline_ = frames_ == 0 ? noise_est : 0.8 * baseline_ + 0.2 * noise_est;
        } else if (noise_est < baseline_ * 1.5) {
            // 이후: 정상 범위(기준선의 1.5배 미만)일 때만 아주 천천히(1%) 갱신.
            // 노이즈가 급증하면(고장 의심) 기준선을 안 올림 → 고장이 "정상"이 되는 걸 방지
            baseline_ = 0.99 * baseline_ + 0.01 * noise_est;
        }
        frames_++;                             // 처리한 프레임 수 증가

        // [노이즈 점수] 기준선 초과분만큼 지수 감점. exp(-x/10) = 부드러운 감점 곡선:
        // 초과 0→1.0점, 10(밝기 단위)→0.37점, 23→0.1점. max(0,..)로 기준선 이하면 감점 없음
        const double noise_score = std::exp(-std::max(0.0, noise_est - baseline_) / 10.0);
        // 최종 품질 = 정지 점수 x 노이즈 점수 (곱셈: 하나만 나빠도 전체가 깎임), 0~1로 자름
        const double quality = std::clamp(freeze_score * noise_score, 0.0, 1.0);
        // 칠판(블랙보드)에 점수 기록 → PublishScores가 읽어감
        config().blackboard->set(name() + "_quality_score", quality);
        // 디버깅용 콘솔 출력
        cout << name() << " quality: " << quality << " (noise " << noise_est
             << ", base " << baseline_ << ", frozen " << frozen_ticks_ << ")" << endl;
        return BT::NodeStatus::SUCCESS;        // 항상 SUCCESS (판정은 점수 소비자가 함)
    }

private:
    rclcpp::Node::SharedPtr node_;             // ROS 노드 핸들 (스마트포인터)
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;   // 구독 객체 (살아있어야 구독 유지)
    sensor_msgs::msg::Image::SharedPtr last_msg_;   // 콜백이 저장하는 최신 프레임
    std::mutex mtx_;                           // last_msg_ 보호용 자물쇠
    std::vector<uint8_t> prev_sample_;         // 직전 tick의 표본 픽셀들 (freeze 비교용)
    int frozen_ticks_ = 0;                     // 프레임이 연속으로 똑같았던 tick 수
    double baseline_ = 0.0;                    // 정상 장면의 이웃 밝기차 기준선
    int frames_ = 0;                           // 지금까지 본 프레임 수 (초반 30개는 학습 구간)
};
