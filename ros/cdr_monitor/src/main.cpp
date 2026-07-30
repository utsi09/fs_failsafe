#include <behaviortree_cpp/basic_types.h>
#include "behaviortree_cpp/bt_factory.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "std_msgs/msg/float64.hpp"
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <csignal>
#include <string>

#include "cdr_monitor/update_system_state.hpp"
#include "behaviortree_ros2/bt_topic_sub_node.hpp"

#include "cdr_monitor/vel_subscriber.hpp"
#include "cdr_monitor/ttc_subscriber.hpp"
#include "cdr_monitor/heading_subscriber.hpp"
#include "cdr_monitor/obstacle_subscriber.hpp"
#include "cdr_monitor/lidar_check.hpp"
#include "cdr_monitor/cam_check.hpp"
#include "cdr_monitor/imu_check.hpp"
#include "cdr_monitor/gps_check.hpp"
#include "cdr_monitor/radar_check.hpp"
#include "cdr_monitor/calculate_ftti.hpp"
#include "cdr_monitor/latency_logger.hpp"
#include "cdr_monitor/lidar_quality_check.hpp"
#include "cdr_monitor/cam_quality_check.hpp"
#include "cdr_monitor/publish_scores.hpp"

using namespace std::chrono_literals;
using namespace BT;

bool g_should_exit = false;
void signal_handler(int) { g_should_exit = true; }

struct SensorTypeInfo {
    std::string type;
    std::string bt_id;
    std::string ftti_port;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc,argv);
    auto shared_node = rclcpp::Node::make_shared("cdr_monitor_node");

    //LatencyLogger::instance().init("/tmp/cdr_latency.csv");

    BehaviorTreeFactory factory;
    std::signal(SIGINT, signal_handler);

    BT::RosNodeParams params;
    params.nh = shared_node;

    factory.registerNodeType<VelSubscriber>("VelSubscriber", params);
    factory.registerNodeType<HeadingSubscriber>("HeadingSubscriber", params);
    factory.registerNodeType<ObstacleSubscriber>("ObstacleSubscriber", params);
    factory.registerNodeType<TtcSubscriber>("TtcSubscriber", params);

    auto blackboard = Blackboard::create();
    blackboard->set("vel_x", 0.0);
    blackboard->set("vel_y", 0.0);
    blackboard->set("current_heading", 0.0);
    blackboard->set("nearest_obstacle", 100.0);
    blackboard->set("ttc_ext_ms", -1.0);
    blackboard->set("important_sensors", std::string(""));

    std::vector<SensorTypeInfo> type_infos = {
        {"lidar",  "LidarChecker", "lidar_ftti"},
        {"camera", "CamChecker",   "cam_ftti"},
        {"radar",  "RadarChecker", "radar_ftti"},
    };

    std::map<std::string, std::vector<std::string>> type_names_map;

    for (const auto& info : type_infos) {
        std::string names_key = "sensors." + info.type + ".names";
        std::vector<std::string> names;
        try {
            shared_node->declare_parameter<std::vector<std::string>>(names_key, std::vector<std::string>{});
            shared_node->get_parameter(names_key, names);
        } catch (const std::exception&) {}
        type_names_map[info.type] = names;

        blackboard->set("lidar_names", type_names_map["lidar"]);
        blackboard->set("camera_names", type_names_map["camera"]);

        for (const auto& name : names) {
            std::string prefix = "sensors." + info.type + "." + name + ".";
            shared_node->declare_parameter<std::string>(prefix + "topic", "");
            std::string topic;
            shared_node->get_parameter(prefix + "topic", topic);
            blackboard->set(name + "_topic", topic);

            if (info.type == "camera") {
                shared_node->declare_parameter<std::string>(
                    prefix + "msg_type", "sensor_msgs/msg/CompressedImage");
                std::string msg_type;
                shared_node->get_parameter(prefix + "msg_type", msg_type);
                blackboard->set(name + "_msgtype", msg_type);
            }
        }
    }

    shared_node->declare_parameter<std::string>("sensors.imu.topic", "");
    shared_node->declare_parameter<std::string>("sensors.imu.frame_id", "");
    shared_node->declare_parameter<std::string>("sensors.gps.topic", "");
    shared_node->declare_parameter<std::string>("sensors.gps.frame_id", "");
    std::string imu_topic, gps_topic;
    shared_node->get_parameter("sensors.imu.topic", imu_topic);
    shared_node->get_parameter("sensors.gps.topic", gps_topic);
    blackboard->set("IMU_topic", imu_topic);
    blackboard->set("GPS_topic", gps_topic);

    blackboard->set("lidar_names", type_names_map["lidar"]);
    blackboard->set("camera_names", type_names_map["camera"]);

    int non_empty_groups = 0;
    for (const auto& info : type_infos) {
        if (!type_names_map[info.type].empty())
            non_empty_groups++;
    }

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

    for (const auto& info : type_infos) {
        const auto& names = type_names_map[info.type];
        if (names.empty()) continue;
        // lidar/camera groups also run a quality checker per sensor
        bool with_quality = (info.type == "lidar" || info.type == "camera");
        size_t group_count = names.size() * (with_quality ? 2 : 1);
        xml += R"(        <Parallel name=")" + info.type + R"(Checks" success_count=")" + std::to_string(group_count) + R"(" failure_count="1">
)";
        for (const auto& name : names) {
            std::string extra_ports;
            if (info.type == "camera")
                extra_ports = R"( msg_type="{@)" + name + R"(_msgtype}")";
            xml += R"(          <Action ID=")" + info.bt_id + R"(" name=")" + name
                + R"(" topic_name="{@)" + name + R"(_topic}")" + extra_ports + " "
                + info.ftti_port + R"(="{@)" + name + R"(_ftti}"/>
)";
            if (info.type == "lidar") {
                xml += R"(          <Action ID="LidarQualityChecker" name=")" + name
                    + R"(" topic_name="{@)" + name + R"(_topic}"/>
)";
            } else if (info.type == "camera") {
                xml += R"(          <Action ID="CamQualityChecker" name=")" + name
                    + R"(" topic_name="{@)" + name + R"(_topic}")" + extra_ports
                    + R"( vel_x="{@vel_x}" vel_y="{@vel_y}"/>
)";
            }
        }
        xml += R"(        </Parallel>
)";
    }

    xml += R"(        <Action ID="ImuChecker" name="IMU" topic_name="{@IMU_topic}" imu_ftti="{@IMU_ftti}"/>
        <Action ID="GpsChecker" name="GPS" topic_name="{@GPS_topic}" gps_ftti="{@GPS_ftti}"/>
)";

    xml += R"(      </Parallel>
      </ForceSuccess>

      <Action ID="PublishScores"/>

    </Sequence>
  </BehaviorTree>
</root>)";

    factory.registerNodeType<UpdateSystemState>("UpdateSystemState", params);
    factory.registerNodeType<CalculateFtti>("CalculateFtti");
    factory.registerNodeType<LidarChecker>("LidarChecker", params);
    factory.registerNodeType<CamChecker>("CamChecker", params);
    factory.registerNodeType<ImuChecker>("ImuChecker", params);
    factory.registerNodeType<GpsChecker>("GpsChecker", params);
    factory.registerNodeType<RadarChecker>("RadarChecker", params);
    factory.registerNodeType<LidarQualityChecker>("LidarQualityChecker", params);
    factory.registerNodeType<CamQualityChecker>("CamQualityChecker", params);
    factory.registerNodeType<PublishScores>("PublishScores", params);

    auto tree = factory.createTreeFromText(xml, blackboard);
    BT::Groot2Publisher groot_publisher(tree, 1667);

    rclcpp::Rate loop_rate(10);
    while(rclcpp::ok() && !g_should_exit) {
        rclcpp::spin_some(shared_node);
        tree.tickOnce();
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
