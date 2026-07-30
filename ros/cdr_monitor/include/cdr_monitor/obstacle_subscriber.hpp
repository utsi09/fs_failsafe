#pragma once
#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include <hesai_zed_fusion/msg/obstacle_array.hpp>
#include <cmath>
#include <iostream>
using namespace std;

class ObstacleSubscriber : public BT::RosTopicSubNode<hesai_zed_fusion::msg::ObstacleArray>
{
public:
    ObstacleSubscriber(const std::string& name, const BT::NodeConfig& conf,
                    const BT::RosNodeParams& params)
                    : RosTopicSubNode(name, conf, params) {}
    static BT::PortsList providedPorts()
    {
        return providedBasicPorts({
            BT::OutputPort<double>("nearest_obstacle"),
        });
    }

    BT::NodeStatus onTick(const std::shared_ptr<hesai_zed_fusion::msg::ObstacleArray>& last_msg) override
    {
        if (last_msg) {
            double min_dist = numeric_limits<double>::max();
            for (const auto &ob : last_msg->obstacles) {
                if (ob.distance < min_dist) {
                    min_dist = ob.distance;
                }
            }
            setOutput("nearest_obstacle", min_dist);
        }
        return BT::NodeStatus::SUCCESS;
    }
};
