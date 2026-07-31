// ============================================================================
// [이 파일의 역할]
// 전체 시스템 흐름: 센서 고장 감시(cdr_monitor 행동트리) → 센서별 점수 계산
//   → PublishScores가 합산 발행 → MRM 슈퍼바이저가 비상 대응 판단.
// 이 파일은 그중 "차량/센서 상태 갱신 + RViz 시각화" 노드의 선언부(헤더)다.
// 매 tick마다: TF(좌표계 변환 시스템)에서 자차 위치·헤딩을 읽어 속도를 계산하고,
// 각 센서의 시야각(FOV) 부채꼴을 마커로 그려 RViz 토픽으로 발행하며,
// 경로가 어느 센서 시야에 들어있는지(중요 센서 판정)를 계산한다.
// 실제 구현(함수 몸통)은 대응하는 .cpp 파일에 있고, 여기는 "설계도"만 있다.
// ============================================================================
#pragma once                                   // 헤더 중복 include 방지
#include "behaviortree_cpp/behavior_tree.h"    // BehaviorTree.CPP 핵심 (SyncActionNode 등)
#include "behaviortree_ros2/ros_node_params.hpp"   // BT 노드에 ROS 핸들을 넘겨주는 구조체
#include <rclcpp/rclcpp.hpp>                   // ROS2 C++ 기본 라이브러리
#include <visualization_msgs/msg/marker_array.hpp>   // RViz에 도형을 그리는 마커 메시지
#include <tf2_ros/buffer.h>                    // TF 버퍼: 좌표 변환 기록을 저장하는 창고
#include <tf2_ros/transform_listener.h>        // TF 리스너: 변환 방송을 받아 버퍼에 채움
#include <cmath>                               // 수학 함수
#include <vector>                              // 동적 배열
#include <string>                              // 문자열
using namespace BT;                            // BT:: 접두사 생략 (SyncActionNode 등)

// 센서 1개의 시야(FOV) 명세를 담는 구조체 (struct = 데이터 묶음 상자)
struct SensorFovSpec {
    std::string name;                          // 센서 이름 (예: "LIDAR_TOP")
    std::string frame_id;                      // 이 센서가 붙어있는 TF 좌표계 이름
    std::string ns;                            // RViz 마커의 네임스페이스(그룹 이름)
    double h_fov_deg;                          // 수평 시야각 (도 단위)
    double range;                              // 감지 거리 (m)
    float r, g, b, a;                          // 마커 색상 (빨강/초록/파랑/투명도, 0~1)
};

// SyncActionNode = "행동트리가 주기적으로 '똑딱(tick)' 부르면 한 번 실행되고
// 즉시 결과를 돌려주는 작업 단위"
class UpdateSystemState : public SyncActionNode {
public:
    // 생성자 선언: 트리 구성 시 한 번 호출됨 (구현은 .cpp에)
    UpdateSystemState(const std::string& name, const NodeConfig& config,
                      const BT::RosNodeParams& params);

    // providedPorts = 트리 XML에서 이 노드로 값을 전달하는 "입력 구멍" 목록 선언
    static PortsList providedPorts();

    // tick() = 트리가 주기적으로 부르는 실행 함수. override = 부모의 가상함수를 덮어씀
    NodeStatus tick() override;

private:
    // 센서 1개의 FOV 부채꼴을 RViz 마커(도형)로 만드는 도우미 함수.
    // is_important(경로가 시야에 있는 센서) / is_failed(고장)에 따라 색이 달라짐
    visualization_msgs::msg::Marker createFovMarker(
        const SensorFovSpec& spec, int id, const rclcpp::Time& stamp,
        bool is_important, bool is_failed);

    // 주어진 경로 점들이 이 센서의 시야(부채꼴) 안에 들어오는지 판정하는 함수
    bool isPathInFov(const std::vector<geometry_msgs::msg::Point>& path_points,
                     const SensorFovSpec& spec);

    std::shared_ptr<rclcpp::Node> node_;       // ROS 노드 핸들 (shared_ptr = 참조 카운트 스마트포인터)
    // Publisher = 토픽(방송 채널)으로 메시지를 내보내는 송신기
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr path_pub_;   // 경로 시각화 마커 발행
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr fov_pub_;    // 센서 FOV 마커 발행

    // TF = ROS의 좌표계 변환 시스템 ("맵 기준 자차 위치가 어디?"를 답해줌).
    // buffer = 변환 기록 창고, listener = 방송을 받아 창고를 채우는 수신기
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    double prev_heading_ = 0.0;                // 직전 tick의 차량 헤딩(방향각) — 변화량 계산용
    double filtered_vel_ = 0.0;                // 저역통과 필터를 거친 속도 (튀는 값 완화)
    rclcpp::Time prev_time_;                   // 직전 tick의 시각 — 시간차(dt) 계산용
    bool first_tick_ = true;                   // 첫 tick 여부 (첫 tick엔 이전 값이 없어 계산 생략)

    std::vector<SensorFovSpec> sensors_;       // 설정에서 읽어온 전체 센서 FOV 목록
};
