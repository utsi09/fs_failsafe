// ============================================================================
// [이 파일의 역할]
// 전체 시스템 흐름: 센서 고장 감시(cdr_monitor 행동트리) → 센서별 점수 계산
//   → PublishScores가 합산해 /monitor/scores, /diagnostics 발행
//   → MRM 슈퍼바이저가 비상정지/우회를 판단.
// 이 파일은 "라이다 데이터가 제때(FTTI 안에) 도착하는가"를 검사하는 행동트리 노드.
// 입력: 라이다 토픽(PointCloud2)의 수신 시각 기록 + XML에서 오는 lidar_ftti(허용 지연 ms).
//   FTTI는 CalculateFtti 노드가 주행 상황(TTC)에 따라 동적으로 계산해 넘겨준다.
// 출력: 블랙보드에 "<이름>_failed"(불리언)와 "<이름>_timeout_score"(0~1) 저장
//   → PublishScores가 읽어감. 품질(내용) 검사는 lidar_quality_check.hpp가 따로 담당.
// ============================================================================
#pragma once                                   // 헤더 중복 include 방지
#include "behaviortree_ros2/bt_topic_sub_node.hpp"   // BT + ROS2 연동 기반 클래스
#include "cdr_monitor/cdr_parser.hpp"          // 직렬화된 메시지에서 타임스탬프만 빼내는 도우미
#include "cdr_monitor/latency_logger.hpp"      // 지연시간 기록용 로거 (실험 데이터 수집)
#include <algorithm>                           // std::clamp 등
#include <chrono>                              // 시계/시간 관련 표준 라이브러리
#include <cmath>                               // 수학 함수
#include <atomic>                              // 원자 변수 (뮤텍스 없이 스레드 안전한 단일 값)
using namespace std;                           // std:: 접두사 생략

// SyncActionNode = "행동트리가 주기적으로 '똑딱(tick)' 부르면 한 번 실행되고
// 즉시 SUCCESS/FAILURE를 돌려주는 작업 단위"
class LidarChecker : public BT::SyncActionNode
{
public:
    // 생성자: 트리 구성 시 한 번 호출됨
    LidarChecker(const std::string& name, const BT::NodeConfig& conf,
                 const BT::RosNodeParams& params)
                 : SyncActionNode(name, conf)  // 부모(BT 노드) 초기화
    {
        // weak_ptr(약한 참조)를 shared_ptr(진짜 소유권)로 승격. 노드가 죽었으면 nullptr
        node_ = params.nh.lock();
        std::string topic;
        // getInput = 트리 XML에서 이 노드로 값을 전달하는 "입력 구멍"에서 값 꺼내기
        if (getInput("topic_name", topic) && !topic.empty()) {
            topic_ = topic;                    // 토픽 이름을 멤버에 보관 (tick에서 유무 확인용)
            // create_generic_subscription = 메시지를 역직렬화(해석)하지 않고
            // "날바이트(직렬화된 상태)" 그대로 받는 구독. 도착 시각만 필요하니 해석 비용을 아낌
            sub_ = node_->create_generic_subscription(
                topic_, "sensor_msgs/msg/PointCloud2",
                rclcpp::SensorDataQoS(),       // 센서용 통신 설정(최신 우선, 유실 허용)
                // 람다(즉석 함수) 콜백: 새 메시지가 올 때마다 세 가지 시각을 기록
                [this](std::shared_ptr<rclcpp::SerializedMessage> msg) {
                    // atomic의 store = 뮤텍스 없이도 안전하게 값을 써넣는 연산.
                    // memory_order_relaxed = "순서 보장은 필요 없고 값만 안 깨지면 됨" (가장 가벼운 모드)
                    // (1) 메시지 헤더에 적힌 생성 시각 (날바이트에서 직접 파싱)
                    last_stamp_ns_.store(cdr_parser::parse_stamp_ns(msg), std::memory_order_relaxed);
                    // (2) ROS 시계 기준 수신 시각 (tick에서 경과시간 계산에 사용)
                    last_recv_ns_.store(node_->now().nanoseconds(), std::memory_order_relaxed);
                    // (3) 벽시계(steady_clock, 절대 되돌아가지 않는 시계) 기준 수신 시각 (로깅용)
                    wall_recv_ns_.store(std::chrono::steady_clock::now().time_since_epoch().count(), std::memory_order_relaxed);
                }
            );
        }
    }

    // 트리 XML에서 받을 수 있는 입력 구멍 목록
    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("topic_name"),   // 감시할 라이다 토픽 이름
            BT::InputPort<double>("lidar_ftti"),        // 허용 지연 한계(ms). CalculateFtti가 동적으로 공급
        };
    }

    // tick() = 트리가 주기적으로 부르는 검사 함수
    BT::NodeStatus tick() override
    {
        if (topic_.empty()) {
            // 토픽이 설정 안 됐으면 검사할 게 없음 → 그냥 통과
            return BT::NodeStatus::SUCCESS;
        }
        double ftti = 850.0;                   // FTTI 기본값 850ms (입력이 없을 때 대비)
        getInput("lidar_ftti", ftti);          // XML 입력 구멍에서 동적 FTTI를 읽어 덮어씀

        // 콜백 스레드가 기록해 둔 "마지막 수신 시각"을 원자적으로 읽음
        int64_t recv_ns = last_recv_ns_.load(std::memory_order_relaxed);
        double timeout = -1.0;                 // -1 = 아직 메시지를 한 번도 못 받음
        if (recv_ns > 0) {
            // 경과 시간(ms) = (지금 - 마지막 수신) / 1e6 (나노초 → 밀리초 변환)
            timeout = (node_->now().nanoseconds() - recv_ns) / 1e6;
        }

        // 고장 판정: 한 번도 못 받았거나, 마지막 수신 후 FTTI를 넘겼으면 실패
        bool failed = (timeout < 0 || timeout > ftti);
        // blackboard = 트리 노드들이 공유하는 칠판(키-값 저장소)에 판정 기록
        config().blackboard->set(name() + "_failed", failed);
        // 연속적인 FTTI 여유 점수: 방금 받았으면 1.0, FTTI 마감에 닿으면 0.0 (선형 감소).
        // 수식: score = 1 - (경과시간 / FTTI), clamp로 0~1 밖은 잘라냄
        double timeout_score = (timeout < 0 || ftti <= 0.0)
            ? 0.0 : std::clamp(1.0 - timeout / ftti, 0.0, 1.0);
        config().blackboard->set(name() + "_timeout_score", timeout_score);   // 점수도 칠판에 기록
        cout << name() << " timeout: " << timeout << "ms / ftti: " << ftti << "ms" << endl;   // 디버깅 출력
        // 실험 분석용: 메시지 생성 시각과 수신 시각을 로거(싱글턴 = 프로그램에 딱 하나 있는 객체)에 기록
        LatencyLogger::instance().log(name(), last_stamp_ns_.load(), wall_recv_ns_.load());
        // 실패면 FAILURE 반환 → 트리 구조에 따라 상위 노드가 대응 분기를 탈 수 있음
        return failed ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
    }

private:
    rclcpp::Node::SharedPtr node_;             // ROS 노드 핸들 (스마트포인터)
    rclcpp::GenericSubscription::SharedPtr sub_;   // 날바이트 구독 객체 (살아있어야 구독 유지)
    // atomic<int64_t> = 64비트 정수를 뮤텍스 없이 스레드 안전하게 읽고 쓰게 해주는 타입
    std::atomic<int64_t> last_stamp_ns_{0};    // 마지막 메시지 헤더의 생성 시각 (ns)
    std::atomic<int64_t> last_recv_ns_{0};     // 마지막 수신 시각, ROS 시계 기준 (ns)
    std::atomic<int64_t> wall_recv_ns_{0};     // 마지막 수신 시각, 벽시계 기준 (ns, 로깅용)
    std::string topic_;                        // 감시 중인 토픽 이름 (비어있으면 검사 안 함)
};
