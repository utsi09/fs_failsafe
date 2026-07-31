/**
 * [main.cpp] cdr_monitor 노드의 시작점(엔트리 포인트).
 *
 * 전체 시스템에서의 역할:
 *   센서(라이다/카메라/레이더/IMU/GPS) 고장을 감시하고,
 *   차 상태(속도·장애물 거리)로 "동적 FTTI"(고장 허용 데드라인)를 계산해서,
 *   각 센서 점수를 /diagnostics 로 발행하는 감시 노드를 띄운다.
 *
 * 데이터 흐름 (매 틱, 초당 10번 반복):
 *   1) Subscribers: 속도/heading/장애물/외부 TTC 토픽을 구독해 최신값 저장
 *   2) UpdateSystemState: 앞으로 갈 경로를 예측 → 그 경로를 보는 "중요 센서" 선정
 *   3) CalculateFtti: FTTI = min(850ms, TTC). 위험이 가까울수록 데드라인을 조인다
 *   4) SensorChecks: 센서마다 "FTTI 안에 새 데이터가 왔나?" + 데이터 품질 검사
 *   5) PublishScores: 검사 결과를 점수로 묶어 /diagnostics 로 발행
 *
 * 이 파일이 하는 일 자체는 "준비 작업": 위 1~5 단계를 행동트리(Behavior Tree)
 * XML 문자열로 조립하고, 노드들을 등록한 뒤, 10Hz 루프에서 트리를 tick 한다.
 */

// ── 외부 라이브러리 헤더들 ──
#include <behaviortree_cpp/basic_types.h>                       // BehaviorTree.CPP 기본 타입들(NodeStatus 등)
#include "behaviortree_cpp/bt_factory.h"                        // 행동트리를 XML로부터 만들어주는 "공장" 클래스
#include <ament_index_cpp/get_package_share_directory.hpp>      // ROS2 패키지 설치 경로를 찾아주는 도우미
#include "behaviortree_cpp/loggers/groot2_publisher.h"          // Groot2(트리 시각화 GUI)에 트리 상태를 쏴주는 로거
#include "std_msgs/msg/float64.hpp"                             // ROS2 표준 메시지: double 하나짜리
#include <rclcpp/rclcpp.hpp>                                    // ROS2 C++ 클라이언트 라이브러리 (노드/토픽/타이머 전부 여기)
#include <iostream>                                             // 콘솔 출력(cout)
#include <csignal>                                              // Ctrl+C 같은 OS 시그널 처리
#include <string>                                               // std::string

// ── 우리 프로젝트의 행동트리 노드 헤더들 ──
#include "cdr_monitor/update_system_state.hpp"                  // "중요 센서" 선정 노드
#include "behaviortree_ros2/bt_topic_sub_node.hpp"              // ROS 토픽 구독을 BT 노드로 감싸주는 베이스 클래스

#include "cdr_monitor/vel_subscriber.hpp"                       // /vel(속도) 구독 노드
#include "cdr_monitor/ttc_subscriber.hpp"                       // /monitor/ttc_ms(외부 TTC) 구독 노드
#include "cdr_monitor/heading_subscriber.hpp"                   // /heading(차 방향) 구독 노드
#include "cdr_monitor/obstacle_subscriber.hpp"                  // 장애물 거리 구독 노드
#include "cdr_monitor/lidar_check.hpp"                          // 라이다 타임아웃 검사 노드
#include "cdr_monitor/cam_check.hpp"                            // 카메라 타임아웃 검사 노드
#include "cdr_monitor/imu_check.hpp"                            // IMU 타임아웃 검사 노드
#include "cdr_monitor/gps_check.hpp"                            // GPS 타임아웃 검사 노드
#include "cdr_monitor/radar_check.hpp"                          // 레이더 타임아웃 검사 노드
#include "cdr_monitor/calculate_ftti.hpp"                       // 동적 FTTI 계산 노드
#include "cdr_monitor/latency_logger.hpp"                       // (실험용) 지연시간 CSV 로거
#include "cdr_monitor/lidar_quality_check.hpp"                  // 라이다 데이터 "품질"(점 개수 등) 검사 노드
#include "cdr_monitor/cam_quality_check.hpp"                    // 카메라 데이터 "품질"(블러/프리즈 등) 검사 노드
#include "cdr_monitor/publish_scores.hpp"                       // 최종 점수를 /diagnostics 로 발행하는 노드

using namespace std::chrono_literals;   // 10ms, 1s 같은 시간 리터럴을 쓰게 해줌
using namespace BT;                     // BT:: 접두사 생략 (BehaviorTree.CPP 네임스페이스)

// Ctrl+C(SIGINT)가 들어오면 true로 바뀌는 전역 플래그.
// 메인 루프가 매번 이 값을 확인해서 깔끔하게 종료한다.
bool g_should_exit = false;
void signal_handler(int) { g_should_exit = true; }   // 시그널이 오면 호출되는 함수

// 센서 "종류"별 설정을 한 줄로 묶어두는 구조체.
// 예: {"lidar", "LidarChecker", "lidar_ftti"}
//   → 설정 파일에서 sensors.lidar.* 를 읽고,
//     행동트리에는 LidarChecker 노드를 만들고,
//     그 노드의 FTTI 입력 포트 이름은 lidar_ftti 라는 뜻.
struct SensorTypeInfo {
    std::string type;       // 설정 파일에서 쓰는 종류 이름 (lidar/camera/radar)
    std::string bt_id;      // 행동트리에 등록된 검사 노드의 ID
    std::string ftti_port;  // 그 노드가 FTTI 값을 받는 포트 이름
};

int main(int argc, char** argv)
{
    // ROS2 시스템 초기화. 모든 ROS2 프로그램의 첫 줄 (통신 준비 시작).
    rclcpp::init(argc,argv);
    // ROS2 "노드"를 하나 만든다. 노드 = ROS 네트워크에 참여하는 하나의 프로그램 단위.
    // make_shared 는 shared_ptr(자동 소유권 관리 포인터)로 만들어 준다:
    // 여러 곳에서 같이 쓰다가, 아무도 안 쓰게 되면 알아서 메모리가 해제된다.
    auto shared_node = rclcpp::Node::make_shared("cdr_monitor_node");

    //LatencyLogger::instance().init("/tmp/cdr_latency.csv");   // (실험용 로거, 지금은 꺼둠)

    // 행동트리 "공장". 우리가 만든 노드 클래스들을 이름표(ID)와 함께 등록해 두면,
    // 나중에 XML 문자열에 그 이름표가 나올 때 실제 객체를 만들어 준다.
    BehaviorTreeFactory factory;
    std::signal(SIGINT, signal_handler);   // Ctrl+C 가 오면 signal_handler 를 불러달라고 OS에 등록

    // ROS 기반 BT 노드들에게 "이 ROS 노드를 통해 통신해라"라고 알려주는 파라미터 묶음.
    BT::RosNodeParams params;
    params.nh = shared_node;   // nh = node handle. 위에서 만든 노드를 넘겨준다.

    // 토픽 구독용 BT 노드 4종을 공장에 등록. (아직 객체가 만들어진 건 아니고 "설계도 등록"만)
    factory.registerNodeType<VelSubscriber>("VelSubscriber", params);
    factory.registerNodeType<HeadingSubscriber>("HeadingSubscriber", params);
    factory.registerNodeType<ObstacleSubscriber>("ObstacleSubscriber", params);
    factory.registerNodeType<TtcSubscriber>("TtcSubscriber", params);

    // "블랙보드" = 행동트리 노드들이 공유하는 화이트보드(키-값 저장소).
    // 한 노드가 값을 써 두면(예: vel_x) 다른 노드가 읽어 갈 수 있다.
    // 아래는 첫 틱에 값이 없어서 에러 나지 않도록 기본값을 미리 적어두는 것.
    auto blackboard = Blackboard::create();
    blackboard->set("vel_x", 0.0);                      // 차 속도 x 성분 [m/s]
    blackboard->set("vel_y", 0.0);                      // 차 속도 y 성분 [m/s]
    blackboard->set("current_heading", 0.0);            // 차가 향한 방향 [rad]
    blackboard->set("nearest_obstacle", 100.0);         // 가장 가까운 장애물 거리 [m] (기본 100m = 멀다)
    blackboard->set("ttc_ext_ms", -1.0);                // 외부(경로 기반) TTC [ms]. -1 = 아직 없음
    blackboard->set("important_sensors", std::string("")); // 중요 센서 목록 (쉼표 구분 문자열)

    // 위에서 정의한 SensorTypeInfo 로 센서 종류 3개를 나열.
    // 이 표를 돌면서 설정 읽기 + XML 조립을 종류별로 한 번에 처리한다.
    std::vector<SensorTypeInfo> type_infos = {
        {"lidar",  "LidarChecker", "lidar_ftti"},
        {"camera", "CamChecker",   "cam_ftti"},
        {"radar",  "RadarChecker", "radar_ftti"},
    };

    // 종류 → 그 종류에 속한 센서 이름 목록 (예: "lidar" → ["Hesai_AT128", ...])
    std::map<std::string, std::vector<std::string>> type_names_map;

    // ── 설정 파일(yaml 파라미터)에서 센서 목록과 토픽 이름을 읽어 블랙보드에 채운다 ──
    for (const auto& info : type_infos) {
        // 예: "sensors.lidar.names" 라는 파라미터 키를 만든다
        std::string names_key = "sensors." + info.type + ".names";
        std::vector<std::string> names;
        try {
            // ROS2 파라미터는 "선언(declare)"을 먼저 해야 "읽기(get)"가 가능하다.
            // 선언 = "이런 이름의 설정값을 받을게요, 기본값은 빈 목록" 이라고 알리는 것.
            shared_node->declare_parameter<std::vector<std::string>>(names_key, std::vector<std::string>{});
            shared_node->get_parameter(names_key, names);   // 실제 값 읽기 (yaml에 있으면 그 값이 옴)
        } catch (const std::exception&) {}   // 이미 선언돼 있어도 죽지 않게 예외는 무시
        type_names_map[info.type] = names;   // 읽은 이름 목록을 표에 저장

        // 라이다/카메라 이름 목록을 블랙보드에도 올려 둔다 (CalculateFtti가 사용)
        blackboard->set("lidar_names", type_names_map["lidar"]);
        blackboard->set("camera_names", type_names_map["camera"]);

        // 이 종류에 속한 센서 하나하나에 대해...
        for (const auto& name : names) {
            // 예: "sensors.lidar.Hesai_AT128." 라는 접두사를 만들고
            std::string prefix = "sensors." + info.type + "." + name + ".";
            shared_node->declare_parameter<std::string>(prefix + "topic", "");   // 토픽 이름 파라미터 선언
            std::string topic;
            shared_node->get_parameter(prefix + "topic", topic);                 // 값 읽기
            blackboard->set(name + "_topic", topic);   // 예: "Hesai_AT128_topic" → "/lidar_points"

            // 카메라만 메시지 타입이 다양해서(압축/비압축) 타입도 설정에서 받는다
            if (info.type == "camera") {
                shared_node->declare_parameter<std::string>(
                    prefix + "msg_type", "sensor_msgs/msg/CompressedImage");
                std::string msg_type;
                shared_node->get_parameter(prefix + "msg_type", msg_type);
                blackboard->set(name + "_msgtype", msg_type);
            }
        }
    }

    // IMU/GPS는 각각 1개뿐이라 목록 없이 토픽만 바로 읽는다
    shared_node->declare_parameter<std::string>("sensors.imu.topic", "");
    shared_node->declare_parameter<std::string>("sensors.imu.frame_id", "");
    shared_node->declare_parameter<std::string>("sensors.gps.topic", "");
    shared_node->declare_parameter<std::string>("sensors.gps.frame_id", "");
    std::string imu_topic, gps_topic;
    shared_node->get_parameter("sensors.imu.topic", imu_topic);
    shared_node->get_parameter("sensors.gps.topic", gps_topic);
    blackboard->set("IMU_topic", imu_topic);
    blackboard->set("GPS_topic", gps_topic);

    // 이름 목록을 블랙보드에 다시 한 번 최종 저장 (루프 안에서 넣은 것의 확정판)
    blackboard->set("lidar_names", type_names_map["lidar"]);
    blackboard->set("camera_names", type_names_map["camera"]);

    // 실제로 센서가 1개라도 있는 종류의 개수를 센다.
    // (예: 레이더가 설정에 없으면 RadarChecks 그룹 자체를 트리에 안 넣기 위해)
    int non_empty_groups = 0;
    for (const auto& info : type_infos) {
        if (!type_names_map[info.type].empty())
            non_empty_groups++;
    }

    // ── 여기부터 행동트리 XML을 "문자열로" 조립한다 ──
    // 행동트리 = 위에서 아래로 순서대로 tick(실행 신호)되는 작업 목록.
    //  - <Sequence>: 자식들을 차례대로 실행, 하나라도 실패하면 멈춤
    //  - <Parallel>: 자식들을 같은 틱에 나란히 실행. success_count 개가 성공하면 성공
    // R"(...)" 는 C++의 raw string: 줄바꿈/따옴표를 그대로 담을 수 있는 문자열 표기법.
    std::string xml = R"(
<root BTCPP_format="4">
  <BehaviorTree ID="MainTree">
    <Sequence name="MainLoop">

      <Parallel name="Subscribers" success_count="4" failure_count="1">
        <Action ID="VelSubscriber" topic_name="/vel" vel_x="{@vel_x}" vel_y="{@vel_y}"/>
        <Action ID="HeadingSubscriber" topic_name="/heading" current_heading="{@current_heading}"/>
        <Action ID="ObstacleSubscriber" topic_name="/hesai_zed/obstacles" nearest_obstacle="{@nearest_obstacle}"/>
        <Action ID="TtcSubscriber" topic_name="/monitor/ttc_ms" ttc_ms="{@ttc_ext_ms}"/>
      </Parallel>

      <Action ID="UpdateSystemState"
              vel_x="{@vel_x}" vel_y="{@vel_y}"
              current_heading="{@current_heading}"
              important_sensors="{@important_sensors}"/>

      <Action ID="CalculateFtti"
              vel_x="{@vel_x}" vel_y="{@vel_y}"
              nearest_obstacle="{@nearest_obstacle}"
              ttc_ext_ms="{@ttc_ext_ms}"
              important_sensors="{@important_sensors}"
              lidar_names="{@lidar_names}" camera_names="{@camera_names}"/>

      <ForceSuccess>
      <Parallel name="SensorChecks" success_count=")" + std::to_string(non_empty_groups + 2) + R"(" failure_count="1">
)";
    // 참고: {@vel_x} 표기 = "블랙보드의 vel_x 칸과 이 포트를 연결해라"는 뜻.
    // <ForceSuccess> = 센서 검사가 실패해도(고장 감지) 트리 전체를 멈추지 않고
    // 다음 단계(PublishScores)로 넘어가게 하는 데코레이터. 고장 "보고"가 목적이지
    // 고장났다고 감시 자체를 멈추면 안 되기 때문.
    // success_count = (센서 그룹 수) + 2(IMU, GPS) → 전부 성공해야 이 Parallel이 성공.

    // ── 설정에 있는 센서 종류/이름마다 검사 노드 XML 을 동적으로 붙인다 ──
    for (const auto& info : type_infos) {
        const auto& names = type_names_map[info.type];
        if (names.empty()) continue;   // 그 종류의 센서가 하나도 없으면 그룹 생략
        // 라이다/카메라는 타임아웃 검사 + 품질 검사, 2개 노드가 센서마다 붙는다
        bool with_quality = (info.type == "lidar" || info.type == "camera");
        size_t group_count = names.size() * (with_quality ? 2 : 1);   // 이 그룹의 자식 노드 총 개수
        // 종류별 Parallel 그룹 시작 태그 (예: <Parallel name="lidarChecks" ...>)
        xml += R"(        <Parallel name=")" + info.type + R"(Checks" success_count=")" + std::to_string(group_count) + R"(" failure_count="1">
)";
        for (const auto& name : names) {
            // 카메라라면 메시지 타입 포트를 추가로 붙인다
            std::string extra_ports;
            if (info.type == "camera")
                extra_ports = R"( msg_type="{@)" + name + R"(_msgtype}")";
            // 타임아웃 검사 노드 한 줄 생성.
            // 예: <Action ID="LidarChecker" name="Hesai_AT128"
            //             topic_name="{@Hesai_AT128_topic}" lidar_ftti="{@Hesai_AT128_ftti}"/>
            xml += R"(          <Action ID=")" + info.bt_id + R"(" name=")" + name
                + R"(" topic_name="{@)" + name + R"(_topic}")" + extra_ports + " "
                + info.ftti_port + R"(="{@)" + name + R"(_ftti}"/>
)";
            // 라이다에는 품질 검사 노드도 하나 더 붙임 (점 개수/밀도 등 확인)
            if (info.type == "lidar") {
                xml += R"(          <Action ID="LidarQualityChecker" name=")" + name
                    + R"(" topic_name="{@)" + name + R"(_topic}"/>
)";
            // 카메라도 품질 검사 노드 추가 (블러/프리즈 판단에 속도가 필요해서 vel 포트도 연결)
            } else if (info.type == "camera") {
                xml += R"(          <Action ID="CamQualityChecker" name=")" + name
                    + R"(" topic_name="{@)" + name + R"(_topic}")" + extra_ports
                    + R"( vel_x="{@vel_x}" vel_y="{@vel_y}"/>
)";
            }
        }
        // 종류별 그룹 닫기
        xml += R"(        </Parallel>
)";
    }

    // IMU/GPS 검사 노드는 항상 1개씩 고정으로 추가
    xml += R"(        <Action ID="ImuChecker" name="IMU" topic_name="{@IMU_topic}" imu_ftti="{@IMU_ftti}"/>
        <Action ID="GpsChecker" name="GPS" topic_name="{@GPS_topic}" gps_ftti="{@GPS_ftti}"/>
)";

    // 트리 마무리: 검사 Parallel 닫고 → 마지막에 점수 발행 노드 → 시퀀스/트리 닫기
    xml += R"(      </Parallel>
      </ForceSuccess>

      <Action ID="PublishScores"/>

    </Sequence>
  </BehaviorTree>
</root>)";

    // XML에 등장하는 나머지 노드 ID들도 공장에 등록.
    // (등록 안 된 ID가 XML에 있으면 createTreeFromText에서 에러가 난다)
    factory.registerNodeType<UpdateSystemState>("UpdateSystemState", params);
    factory.registerNodeType<CalculateFtti>("CalculateFtti");   // ROS 통신이 없어서 params 불필요
    factory.registerNodeType<LidarChecker>("LidarChecker", params);
    factory.registerNodeType<CamChecker>("CamChecker", params);
    factory.registerNodeType<ImuChecker>("ImuChecker", params);
    factory.registerNodeType<GpsChecker>("GpsChecker", params);
    factory.registerNodeType<RadarChecker>("RadarChecker", params);
    factory.registerNodeType<LidarQualityChecker>("LidarQualityChecker", params);
    factory.registerNodeType<CamQualityChecker>("CamQualityChecker", params);
    factory.registerNodeType<PublishScores>("PublishScores", params);

    // 조립한 XML 문자열 + 블랙보드로 실제 행동트리 객체를 생성
    auto tree = factory.createTreeFromText(xml, blackboard);
    // Groot2(트리 시각화 프로그램)가 1667 포트로 접속해 실시간 상태를 볼 수 있게 함
    BT::Groot2Publisher groot_publisher(tree, 1667);

    // ── 메인 루프: 초당 10번 ──
    rclcpp::Rate loop_rate(10);   // "10Hz 로 돌려라"라는 속도 조절기
    while(rclcpp::ok() && !g_should_exit) {   // ROS가 살아있고 Ctrl+C가 안 눌린 동안
        rclcpp::spin_some(shared_node);   // 도착해 있는 토픽 메시지들의 콜백을 한 번 처리
        tree.tickOnce();                  // 행동트리를 루트부터 한 번 tick (위의 1~5 단계 실행)
        loop_rate.sleep();                // 다음 틱까지 남은 시간만큼 잠들어 10Hz 유지
    }

    // 종료 정리: ROS2 통신을 내려놓고 프로그램 끝
    rclcpp::shutdown();
    return 0;
}
