// ============================================================================
// [이 파일의 역할] — 동적 FTTI 계산기의 선언부(헤더)
// FTTI(Fault Tolerant Time Interval) = "센서가 이만큼 늦어도 아직 안전한 최대
// 허용 지연 시간". 전체 시스템 흐름: 이 노드가 자차 속도와 앞차와의 거리로
// TTC(Time To Collision, 충돌까지 남은 시간)를 계산 → 상황이 위험할수록 짧은
// FTTI를 산출 → 블랙보드를 통해 각 센서 타임아웃 검사 노드(lidar_check,
// cam_check 등)의 "*_ftti" 입력 구멍으로 전달된다.
// 즉, "위험할 땐 센서 지연을 조금만 봐주고, 한가할 땐 넉넉히 봐주는" 조절 장치.
// 실제 구현(함수 몸통)은 대응하는 .cpp 파일에 있고, 여기는 설계도만 있다.
// ============================================================================
#pragma once                                   // 헤더 중복 include 방지
#include "behaviortree_cpp/behavior_tree.h"    // BehaviorTree.CPP 핵심 (SyncActionNode 등)
#include "behaviortree_ros2/ros_node_params.hpp"   // BT 노드에 ROS 핸들을 넘겨주는 구조체
#include <rclcpp/rclcpp.hpp>                   // ROS2 C++ 기본 라이브러리
#include <visualization_msgs/msg/marker_array.hpp>   // RViz 시각화 마커 메시지
#include <tf2_ros/buffer.h>                    // TF(좌표 변환) 버퍼
#include <tf2_ros/transform_listener.h>        // TF 방송 수신기
#include <chrono>                              // 시계/시간 라이브러리
#include <cmath>                               // 수학 함수
#include <vector>                              // 동적 배열
#include <string>                              // 문자열
using namespace BT;                            // BT:: 접두사 생략
using namespace std;                           // std:: 접두사 생략
// SyncActionNode = "행동트리가 주기적으로 '똑딱(tick)' 부르면 한 번 실행되고
// 즉시 결과를 돌려주는 작업 단위"
class CalculateFtti : public SyncActionNode {
public:
    // 생성자 선언: 트리 구성 시 한 번 호출 (구현은 .cpp에)
    CalculateFtti(const std::string& name, const NodeConfig& config);

    // providedPorts = 트리 XML에서 이 노드로 값을 전달하는 "입력 구멍" 목록 선언
    // (속도, 장애물 거리 등을 받고, 계산된 FTTI를 출력 구멍으로 내보냄)
    static PortsList providedPorts();

    // tick() = 트리가 주기적으로 부르는 실행 함수. 매 tick TTC → FTTI 계산
    NodeStatus tick() override;

private:
    // 속도 벡터 (vel_x, vel_y)의 크기(속력)를 구하는 도우미: sqrt(vx^2 + vy^2)
    double calculate_vel(double vel_x, double vel_y);
    // TTC = 거리 / 속력 ("이 속도로 계속 가면 몇 초 뒤에 부딪히나")
    double calculate_ttc(double vel, double dist);
    // XML 포트로 들어온 원시 문자열(예: "a;b;c")을 조각 목록으로 쪼개는 파서
    vector<string> parsing_port(string raw);

    // 센서가 건강했을 때 마지막으로 잰 TTC. 센서가 죽으면 새 TTC를 믿을 수 없으니
    // 이 "신뢰된 값"을 대신 쓴다. 1000.0 = 사실상 무한대(위험 없음) 초기값
    double trusted_ttc_ = 1000.0;   // last TTC measured while sources healthy
    bool was_unhealthy_ = false;               // 직전 tick에 소스 센서가 불건강했는지 (상태 전환 감지용)
    // 불건강 상태가 시작된 시각 (steady_clock = 절대 뒤로 가지 않는 시계)
    std::chrono::steady_clock::time_point unhealthy_since_;
};
