#pragma once
#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include "cdr_monitor/cdr_parser.hpp"
#include "cdr_monitor/latency_logger.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <atomic>
using namespace std;

class CamChecker : public BT::SyncActionNode
{
public:
    CamChecker(const std::string& name, const BT::NodeConfig& conf,
               const BT::RosNodeParams& params)
               : SyncActionNode(name, conf)
    {
        node_ = params.nh.lock();
        std::string topic;
        std::string msg_type = "sensor_msgs/msg/CompressedImage";
        getInput("msg_type", msg_type);
        if (msg_type.empty()) msg_type = "sensor_msgs/msg/CompressedImage";
        if (getInput("topic_name", topic) && !topic.empty()) {
            topic_ = topic;
            sub_ = node_->create_generic_subscription(
                topic_, msg_type,
                rclcpp::SensorDataQoS(),
                [this](std::shared_ptr<rclcpp::SerializedMessage> msg) {
                    last_stamp_ns_.store(cdr_parser::parse_stamp_ns(msg), std::memory_order_relaxed);
                    last_recv_ns_.store(node_->now().nanoseconds(), std::memory_order_relaxed);
                    wall_recv_ns_.store(std::chrono::steady_clock::now().time_since_epoch().count(), std::memory_order_relaxed);
                }
            );
        }
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("topic_name"),
            BT::InputPort<std::string>("msg_type", "sensor_msgs/msg/CompressedImage", "ROS message type of the topic"),
            BT::InputPort<double>("cam_ftti"),
        };
    }

    BT::NodeStatus tick() override
    {
        if (topic_.empty()) {
            return BT::NodeStatus::SUCCESS;
        }
        double ftti = 850.0;
        getInput("cam_ftti", ftti);

        int64_t recv_ns = last_recv_ns_.load(std::memory_order_relaxed);
        double timeout = -1.0;
        if (recv_ns > 0) {
            timeout = (node_->now().nanoseconds() - recv_ns) / 1e6;
        }

        bool failed = (timeout < 0 || timeout > ftti);
        config().blackboard->set(name() + "_failed", failed);
        // continuous FTTI margin: 1.0 fresh, 0.0 at/after the FTTI deadline
        double timeout_score = (timeout < 0 || ftti <= 0.0)
            ? 0.0 : std::clamp(1.0 - timeout / ftti, 0.0, 1.0);
        config().blackboard->set(name() + "_timeout_score", timeout_score);
        cout << name() << " timeout: " << timeout << "ms / ftti: " << ftti << "ms" << endl;
        LatencyLogger::instance().log(name(), last_stamp_ns_.load(), wall_recv_ns_.load());
        return failed ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::GenericSubscription::SharedPtr sub_;
    std::atomic<int64_t> last_stamp_ns_{0};
    std::atomic<int64_t> last_recv_ns_{0};
    std::atomic<int64_t> wall_recv_ns_{0};
    std::string topic_;
};
