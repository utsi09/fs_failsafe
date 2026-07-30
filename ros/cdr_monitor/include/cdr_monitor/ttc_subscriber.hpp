#pragma once
#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include <std_msgs/msg/float64.hpp>

// External spatio-temporal TTC [ms], computed by the adapter from the planned
// ego trajectory crossed with the perception module's predicted object paths.
// Negative value = no predicted encounter (kinematic TTC is used alone).
class TtcSubscriber : public BT::RosTopicSubNode<std_msgs::msg::Float64>
{
public:
    TtcSubscriber(const std::string& name, const BT::NodeConfig& conf,
                  const BT::RosNodeParams& params)
                  : RosTopicSubNode(name, conf, params) {}

    static BT::PortsList providedPorts()
    {
        return providedBasicPorts({
            BT::OutputPort<double>("ttc_ms"),
        });
    }

    BT::NodeStatus onTick(const std::shared_ptr<std_msgs::msg::Float64>& last_msg) override
    {
        if (last_msg) {
            setOutput("ttc_ms", last_msg->data);
        }
        return BT::NodeStatus::SUCCESS;
    }
};
