// ============================================================================
// [이 파일의 역할]
// 전체 시스템 흐름: 센서 고장 감시(cdr_monitor의 행동트리) → 센서별 점수 계산
//   → PublishScores가 점수를 합산해 /monitor/scores, /diagnostics로 발행
//   → MRM 슈퍼바이저가 그 점수를 보고 비상정지/우회 같은 MRM을 판단.
// 이 파일은 그중 "라이다 데이터의 품질"을 0~1 점수로 매기는 행동트리 노드다.
// 데이터 입력: 라이다 포인트클라우드 토픽(sensor_msgs/PointCloud2)을 구독.
// 데이터 출력: 블랙보드(트리 공유 저장소)에 "<노드이름>_quality_score" 키로 저장
//   → PublishScores 노드가 이 값을 읽어감.
// 타임아웃(FTTI) 검사는 lidar_check.hpp가 따로 담당하고, 여기는 "데이터가 오긴
// 오는데 내용이 이상한가"(노이즈 증가, 포인트 수 감소)만 본다.
// ============================================================================
#pragma once                                   // 이 헤더 파일이 두 번 include돼도 한 번만 처리되게 하는 지시문
#include "behaviortree_ros2/bt_topic_sub_node.hpp"   // BehaviorTree.CPP + ROS2 연동 기반 클래스들
#include <sensor_msgs/msg/point_cloud2.hpp>    // 라이다 포인트클라우드 메시지 타입 정의
#include <algorithm>                           // std::max, std::clamp 등
#include <cmath>                               // sqrt, exp, fabs 등 수학 함수
#include <mutex>                               // 뮤텍스(스레드 자물쇠)
using namespace std;                           // std:: 접두사를 생략할 수 있게 함

// [원리 요약] 라이다 품질을 실제 데이터로 추정한다 (타임아웃 검사와는 독립):
//  - 가우시안 거리 노이즈가 생기면 → 이웃 포인트 간 거리 차이가 커짐 (noise_score 하락)
//  - 포인트 드롭아웃이 생기면 → 포인트 개수가 줄어듦 (count_score 하락)
//  - 최종 품질 = noise_score * count_score (둘 다 1이어야 1점)
// SyncActionNode = "행동트리가 주기적으로 이 노드를 '똑딱(tick)' 하고 부르면
// 한 번 실행되고 즉시 결과(SUCCESS/FAILURE)를 돌려주는 작업 단위" (BT의 기본 부품)
class LidarQualityChecker : public BT::SyncActionNode
{
public:
    // 생성자: 트리가 만들어질 때 딱 한 번 호출됨.
    // name = 트리 XML에서 붙인 이 노드의 이름, conf = 트리 설정(블랙보드 포함),
    // params = ROS 노드 핸들 등 ROS 관련 정보 묶음
    LidarQualityChecker(const std::string& name, const BT::NodeConfig& conf,
                        const BT::RosNodeParams& params)
                        : SyncActionNode(name, conf)   // 부모 클래스(BT 노드) 초기화
    {
        // weak_ptr(약한 참조)를 shared_ptr(진짜 소유권)로 승격.
        // ROS 노드가 이미 죽었으면 nullptr가 됨
        node_ = params.nh.lock();
        std::string topic;                     // 구독할 토픽 이름을 담을 변수
        // getInput = 트리 XML에서 이 노드로 값을 전달하는 "입력 구멍"에서 값을 꺼냄.
        // topic_name이 XML에 적혀 있고 비어있지 않을 때만 구독을 시작
        if (getInput("topic_name", topic) && !topic.empty()) {
            // create_subscription = "이 토픽(방송 채널)에 새 데이터가 올 때마다
            // 아래 람다(즉석에서 만든 이름 없는 함수)를 실행해달라"고 ROS에 등록.
            // SensorDataQoS = 센서용 통신 설정(최신 데이터 우선, 유실 허용)
            sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
                topic, rclcpp::SensorDataQoS(),
                // [this](...) = this(이 객체)를 캡처한 람다. 콜백 안에서 멤버 변수 접근 가능
                [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                    // lock_guard = 뮤텍스 자물쇠를 잠그는 도우미. 이 블록이 끝나면 자동 해제.
                    // 콜백 스레드와 tick() 스레드가 last_msg_를 동시에 만지는 걸 방지
                    std::lock_guard<std::mutex> lk(mtx_);
                    last_msg_ = msg;           // 가장 최근 메시지를 저장 (tick에서 분석용)
                });
        }
    }

    // providedPorts = "이 노드가 XML로부터 받을 수 있는 입력 구멍 목록"을 트리에 알려주는 함수
    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("topic_name"),   // 구독할 라이다 토픽 이름
        };
    }

    // tick() = 행동트리가 주기적으로 "똑딱" 하고 부르는 함수. 매 호출마다 품질을 한 번 계산
    BT::NodeStatus tick() override
    {
        sensor_msgs::msg::PointCloud2::SharedPtr msg;   // 분석할 메시지를 담을 로컬 변수
        {
            // 자물쇠를 잠근 채 최신 메시지 포인터만 복사해 옴 (잠금 시간 최소화).
            // 블록 { }을 벗어나면 lock_guard가 자동으로 자물쇠를 풂
            std::lock_guard<std::mutex> lk(mtx_);
            msg = last_msg_;
        }
        if (!msg) {
            // 아직 데이터가 한 번도 안 왔음 → 품질은 1.0(만점)으로 둔다.
            // "데이터가 없는 것" 자체는 타임아웃 검사(lidar_check)가 담당하므로 여기선 관여 안 함.
            // blackboard = 트리 노드들이 공유하는 칠판(키-값 저장소). 거기에 점수를 적어둠
            config().blackboard->set(name() + "_quality_score", 1.0);
            return BT::NodeStatus::SUCCESS;    // 이 노드는 항상 SUCCESS (판정은 점수로 전달)
        }

        // PointCloud2는 "바이트 덩어리 + 필드 설명서" 구조.
        // x, y, z 좌표가 각 포인트의 몇 번째 바이트에 있는지(offset)를 설명서에서 찾는다
        int ox = -1, oy = -1, oz = -1;         // -1 = 아직 못 찾음
        for (const auto& f : msg->fields) {    // 필드 설명서를 하나씩 훑음
            if (f.datatype != sensor_msgs::msg::PointField::FLOAT32) continue;   // float32가 아니면 건너뜀
            if (f.name == "x") ox = f.offset;          // x 좌표의 바이트 위치
            else if (f.name == "y") oy = f.offset;     // y 좌표의 바이트 위치
            else if (f.name == "z") oz = f.offset;     // z 좌표의 바이트 위치
        }
        // 전체 포인트 개수 = 가로 x 세로 (라이다는 보통 height=1 이라 width가 곧 개수)
        const size_t n = static_cast<size_t>(msg->width) * msg->height;
        if (ox < 0 || oy < 0 || oz < 0 || n < 100) {
            // x/y/z를 못 찾았거나 포인트가 100개 미만이면 통계를 낼 수 없음 → 만점 처리하고 넘어감
            config().blackboard->set(name() + "_quality_score", 1.0);
            return BT::NodeStatus::SUCCESS;
        }

        // [노이즈 추정] 연속한 두 포인트의 거리(range) 차이로 표준편차 sigma를 추정.
        // 수학적 근거: 서로 독립인 가우시안 노이즈 r1, r2에 대해 E|r1 - r2| = 2*sigma/sqrt(pi).
        // 즉 |차이|의 평균에 sqrt(pi)/2 (~0.886)를 곱하면 sigma가 복원된다.
        // stride = 포인트가 많으면 다 보지 않고 건너뛰며 약 4000쌍만 표본 추출 (계산량 절약)
        const size_t stride = std::max<size_t>(1, n / 4000);
        double acc = 0.0;                      // |거리 차이|의 누적 합
        size_t cnt = 0;                        // 표본 개수
        // range_at = i번째 포인트의 원점 거리(range)를 구하는 람다(즉석 함수).
        // [&] = 바깥 변수들(msg, ox 등)을 참조로 빌려 씀
        auto range_at = [&](size_t i) {
            // point_step = 포인트 1개가 차지하는 바이트 수. i번째 포인트의 시작 주소를 계산
            const uint8_t* p = &msg->data[i * msg->point_step];
            float x, y, z;
            // 바이트 덩어리에서 float 4바이트씩 복사해 꺼냄 (memcpy = 안전한 바이트 복사)
            memcpy(&x, p + ox, 4); memcpy(&y, p + oy, 4); memcpy(&z, p + oz, 4);
            return std::sqrt(x * x + y * y + z * z);   // 원점에서의 거리 = sqrt(x^2+y^2+z^2)
        };
        for (size_t i = 0; i + 1 < n; i += stride) {   // stride 간격으로 이웃 쌍 (i, i+1)을 표본 추출
            const double r1 = range_at(i), r2 = range_at(i + 1);
            // NaN/무한대이거나 0.5m 미만(자차 몸체 반사 등)이면 표본에서 제외
            if (!std::isfinite(r1) || !std::isfinite(r2) || r1 < 0.5 || r2 < 0.5) continue;
            const double dr = std::fabs(r1 - r2);      // 이웃 거리 차이의 절댓값
            if (dr < 1.0) { acc += dr; cnt++; }   // 1m 이상 차이는 진짜 물체 경계(depth edge)이므로 노이즈 표본에서 제외
        }
        // sigma 추정치 = 평균|차이| * 0.886.  0.886 = sqrt(pi)/2 (위 공식의 역변환 계수).
        // 표본이 20개 이하면 신뢰할 수 없으니 0으로 둠
        const double sigma_est = cnt > 20 ? (acc / cnt) * 0.886 : 0.0;  // sqrt(pi)/2

        // [베이스라인 학습] 장면 자체의 자연스러운 울퉁불퉁함(구조적 거칠기)을 노이즈로
        // 오해하지 않도록, "깨끗할 때의 sigma"를 천천히 따라가는 기준선을 유지한다.
        // 갑작스런(고장 같은) 점프에는 기준선이 따라가지 않게 설계됨
        if (frames_ < 30) {
            // 처음 30프레임: 빠르게 학습 (지수이동평균, 새 값 반영 비율 20%)
            baseline_ = frames_ == 0 ? sigma_est : 0.8 * baseline_ + 0.2 * sigma_est;
        } else if (sigma_est < baseline_ * 1.5) {
            // 그 이후: 현재값이 기준선의 1.5배 미만(정상 범위)일 때만 아주 천천히(1%) 갱신.
            // 노이즈가 갑자기 커지면(고장 의심) 기준선을 절대 안 올림 → 고장이 "정상화"되는 걸 방지
            baseline_ = 0.99 * baseline_ + 0.01 * sigma_est;
        }
        frames_++;                             // 지금까지 처리한 프레임 수 증가

        // [노이즈 점수] 기준선 초과분만큼 지수적으로 감점.
        // exp(-x/0.1) = 부드러운 감점 곡선: 초과 0m→1.0점, 0.1m→0.37점, 0.23m→0.1점.
        // max(0, ...) = 기준선보다 좋으면(초과분 음수) 감점 없음
        const double noise_score = std::exp(-std::max(0.0, sigma_est - baseline_) / 0.1);

        // [포인트 개수 기준선] 같은 방식으로 "정상일 때의 포인트 수"를 학습
        if (frames_ < 30) {
            // 초반: 빠르게 학습 (0이면 첫 값으로 초기화)
            count_baseline_ = count_baseline_ == 0.0 ? double(n)
                                                     : 0.8 * count_baseline_ + 0.2 * n;
        } else if (n > count_baseline_ * 0.7) {
            // 이후: 개수가 기준선의 70% 이상(정상)일 때만 천천히 갱신.
            // 드롭아웃으로 개수가 확 줄면 기준선을 안 내림 → 고장 상태가 기준이 되는 걸 방지
            count_baseline_ = 0.99 * count_baseline_ + 0.01 * n;
        }
        // [개수 점수] 현재 개수 / 정상 개수, 0~1로 잘라냄(clamp).
        // 포인트가 절반으로 줄면 0.5점. 기준선이 아직 없으면 만점
        const double count_score =
            count_baseline_ > 0 ? std::clamp(n / count_baseline_, 0.0, 1.0) : 1.0;

        // 최종 품질 = 노이즈 점수 x 개수 점수 (곱셈이라 어느 한쪽만 나빠도 전체가 깎임)
        const double quality = std::clamp(noise_score * count_score, 0.0, 1.0);
        // 계산한 품질 점수를 칠판(블랙보드)에 기록 → PublishScores가 읽어감
        config().blackboard->set(name() + "_quality_score", quality);
        // 디버깅용 콘솔 출력 (점수, 추정 sigma, 기준선, 포인트 수)
        cout << name() << " quality: " << quality << " (sigma " << sigma_est
             << "m, base " << baseline_ << "m, pts " << n << ")" << endl;
        return BT::NodeStatus::SUCCESS;        // 항상 SUCCESS 반환 (실패 판정은 점수 소비자가 함)
    }

private:
    rclcpp::Node::SharedPtr node_;             // ROS 노드 핸들 (shared_ptr = 참조 카운트 방식 스마트포인터)
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;   // 구독 객체 (이게 살아있어야 구독 유지)
    sensor_msgs::msg::PointCloud2::SharedPtr last_msg_;   // 콜백이 저장하는 최신 메시지
    std::mutex mtx_;                           // last_msg_를 보호하는 자물쇠 (콜백 스레드 vs tick 스레드)
    double baseline_ = 0.0;                    // 정상 상태의 sigma 기준선 (지수이동평균)
    double count_baseline_ = 0.0;              // 정상 상태의 포인트 개수 기준선
    int frames_ = 0;                           // 지금까지 본 프레임 수 (초반 30프레임은 학습 구간)
};
