// ============================================================================
// [이 파일의 역할]
// 전체 시스템 흐름: 센서 고장 감시(cdr_monitor 행동트리) → 센서별 점수 계산
//   → PublishScores가 합산해 /monitor/scores, /diagnostics 발행
//   → MRM 슈퍼바이저가 비상정지/우회를 판단.
// 이 파일은 "레이더 데이터가 제때(FTTI 안에) 도착하는가"를 검사하는 행동트리 노드.
// 레이더도 PointCloud2 타입 토픽으로 발행되므로 라이다 검사와 구조가 같지만,
// timeout_score(연속 점수) 없이 failed(고장 여부)만 블랙보드에 기록하는 단순 버전.
// 입력: 레이더 토픽 수신 시각 + XML의 radar_ftti(ms, CalculateFtti가 동적 계산).
// 출력: 블랙보드의 "<이름>_failed".
// ============================================================================
#pragma once                                   // 헤더 중복 include 방지
#include "behaviortree_ros2/bt_topic_sub_node.hpp"   // BT + ROS2 연동 기반 클래스
#include "cdr_monitor/cdr_parser.hpp"          // 직렬화 메시지에서 타임스탬프 파싱 도우미
#include "cdr_monitor/latency_logger.hpp"      // 지연시간 기록 로거
#include <chrono>                              // 시계/시간 라이브러리
#include <cmath>                               // 수학 함수
#include <atomic>                              // 원자 변수 (뮤텍스 없는 스레드 안전 값)
using namespace std;                           // std:: 접두사 생략

// SyncActionNode = "행동트리가 주기적으로 '똑딱(tick)' 부르면 한 번 실행되고
// 즉시 SUCCESS/FAILURE를 돌려주는 작업 단위"
class RadarChecker : public BT::SyncActionNode
{
public:
    // 생성자: 트리 구성 시 한 번 호출
    RadarChecker(const std::string& name, const BT::NodeConfig& conf,
                 const BT::RosNodeParams& params)
                 : SyncActionNode(name, conf)  // 부모(BT 노드) 초기화
    {
        // weak_ptr(약한 참조)를 shared_ptr(진짜 소유권)로 승격. 노드가 죽었으면 nullptr
        node_ = params.nh.lock();
        std::string topic;
        // getInput = 트리 XML에서 값을 전달받는 "입력 구멍"에서 토픽 이름 꺼내기
        if (getInput("topic_name", topic) && !topic.empty()) {
            topic_ = topic;                    // 토픽 이름 보관
            // create_generic_subscription = 메시지를 해석하지 않고 날바이트로 받는 구독
            // (도착 시각만 필요하므로 해석 비용 절약)
            sub_ = node_->create_generic_subscription(
                topic_, "sensor_msgs/msg/PointCloud2",
                rclcpp::SensorDataQoS(),       // 센서용 통신 설정(최신 우선, 유실 허용)
                // 람다(즉석 함수) 콜백: 메시지가 올 때마다 시각 3종 기록
                [this](std::shared_ptr<rclcpp::SerializedMessage> msg) {
                    // atomic store + memory_order_relaxed = 뮤텍스 없이 값만 안전하게 쓰는 가장 가벼운 방식
                    last_stamp_ns_.store(cdr_parser::parse_stamp_ns(msg), std::memory_order_relaxed);   // 메시지 헤더의 생성 시각
                    last_recv_ns_.store(node_->now().nanoseconds(), std::memory_order_relaxed);         // ROS 시계 기준 수신 시각
                    wall_recv_ns_.store(std::chrono::steady_clock::now().time_since_epoch().count(), std::memory_order_relaxed);   // 벽시계 기준 수신 시각 (로깅용)
                }
            );
        }
    }

    // 트리 XML에서 받을 수 있는 입력 구멍 목록
    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("topic_name"),   // 감시할 레이더 토픽 이름
            BT::InputPort<double>("radar_ftti"),        // 허용 지연 한계(ms). CalculateFtti가 동적 공급
        };
    }

    // tick() = 트리가 주기적으로 부르는 검사 함수
    BT::NodeStatus tick() override
    {
        if (topic_.empty()) {
            // 토픽 미설정 → 검사 대상 없음, 통과
            return BT::NodeStatus::SUCCESS;
        }
        double ftti = 850.0;                   // FTTI 기본값 850ms
        getInput("radar_ftti", ftti);          // XML에서 동적 FTTI를 읽어 덮어씀

        // 콜백이 기록한 마지막 수신 시각을 원자적으로 읽음
        int64_t recv_ns = last_recv_ns_.load(std::memory_order_relaxed);
        double timeout = -1.0;                 // -1 = 아직 한 번도 못 받음
        if (recv_ns > 0) {
            // 마지막 수신 후 경과 시간(ms) = 나노초 차이 / 1e6
            timeout = (node_->now().nanoseconds() - recv_ns) / 1e6;
        }

        // 고장 판정: 한 번도 못 받았거나 FTTI를 초과했으면 실패
        bool failed = (timeout < 0 || timeout > ftti);
        // blackboard = 트리 노드들이 공유하는 칠판(키-값 저장소)에 판정 기록
        config().blackboard->set(name() + "_failed", failed);
        cout << name() << " timeout: " << timeout << "ms / ftti: " << ftti << "ms" << endl;   // 디버깅 출력
        // 실험용 지연 기록 (싱글턴 로거)
        LatencyLogger::instance().log(name(), last_stamp_ns_.load(), wall_recv_ns_.load());
        // 실패면 FAILURE → 상위 트리가 대응 분기 가능
        return failed ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
    }

private:
    rclcpp::Node::SharedPtr node_;             // ROS 노드 핸들 (스마트포인터)
    rclcpp::GenericSubscription::SharedPtr sub_;   // 날바이트 구독 객체
    // atomic = 뮤텍스 없이 스레드 안전하게 읽고 쓰는 정수
    std::atomic<int64_t> last_stamp_ns_{0};    // 마지막 메시지의 생성 시각 (ns)
    std::atomic<int64_t> last_recv_ns_{0};     // 마지막 수신 시각, ROS 시계 (ns)
    std::atomic<int64_t> wall_recv_ns_{0};     // 마지막 수신 시각, 벽시계 (ns, 로깅용)
    std::string topic_;                        // 감시 중인 토픽 이름
};
