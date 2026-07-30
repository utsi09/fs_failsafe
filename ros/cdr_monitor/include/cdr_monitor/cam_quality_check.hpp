#pragma once
#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <algorithm>
#include <cmath>
#include <mutex>
#include <vector>
using namespace std;

// Estimates image quality (0~1) from the actual pixels: gaussian pixel noise
// raises the horizontal neighbor difference, a frozen image makes consecutive
// frames identical. quality = freeze_score * noise_score.
// Only raw sensor_msgs/Image is analyzed (compressed inputs return 1.0).
class CamQualityChecker : public BT::SyncActionNode
{
public:
    CamQualityChecker(const std::string& name, const BT::NodeConfig& conf,
                      const BT::RosNodeParams& params)
                      : SyncActionNode(name, conf)
    {
        node_ = params.nh.lock();
        std::string topic, msg_type;
        getInput("msg_type", msg_type);
        if (getInput("topic_name", topic) && !topic.empty() &&
            msg_type == "sensor_msgs/msg/Image") {
            sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
                topic, rclcpp::SensorDataQoS(),
                [this](sensor_msgs::msg::Image::SharedPtr msg) {
                    std::lock_guard<std::mutex> lk(mtx_);
                    last_msg_ = msg;
                });
        }
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("topic_name"),
            BT::InputPort<std::string>("msg_type", "sensor_msgs/msg/CompressedImage", "ROS message type of the topic"),
            BT::InputPort<double>("vel_x", 0.0, "ego velocity for standstill gating"),
            BT::InputPort<double>("vel_y", 0.0, "ego velocity for standstill gating"),
        };
    }

    BT::NodeStatus tick() override
    {
        sensor_msgs::msg::Image::SharedPtr msg;
        {
            std::lock_guard<std::mutex> lk(mtx_);
            msg = last_msg_;
        }
        if (!msg || msg->width < 8 || msg->height < 8) {
            config().blackboard->set(name() + "_quality_score", 1.0);
            return BT::NodeStatus::SUCCESS;
        }

        const size_t pixel_stride = msg->step / msg->width;   // 4 for bgra8
        const size_t grid = 64;
        const size_t sx = std::max<size_t>(1, msg->width / grid);
        const size_t sy = std::max<size_t>(1, msg->height / grid);

        std::vector<uint8_t> sample;
        sample.reserve(grid * grid);
        double noise_acc = 0.0;
        size_t noise_cnt = 0;
        for (size_t y = 0; y < msg->height; y += sy) {
            const uint8_t* row = &msg->data[y * msg->step];
            for (size_t x = 0; x + 1 < msg->width; x += sx) {
                const uint8_t v = row[x * pixel_stride];           // one channel is enough
                sample.push_back(v);
                noise_acc += std::fabs(double(v) - double(row[(x + 1) * pixel_stride]));
                noise_cnt++;
            }
        }
        const double noise_est = noise_cnt ? noise_acc / noise_cnt : 0.0;

        // freeze detection: identical consecutive frames for several ticks.
        // Gated by ego speed: a stopped vehicle in a static scene produces
        // identical frames legitimately (e.g. waiting at a red light).
        double vx = 0.0, vy = 0.0;
        getInput("vel_x", vx);
        getInput("vel_y", vy);
        const bool moving = std::hypot(vx, vy) >= 0.5;   // m/s
        double freeze_score = 1.0;
        if (prev_sample_.size() == sample.size()) {
            double diff = 0.0;
            for (size_t i = 0; i < sample.size(); i++)
                diff += std::fabs(double(sample[i]) - double(prev_sample_[i]));
            diff /= sample.size();
            if (moving) frozen_ticks_ = diff < 0.3 ? frozen_ticks_ + 1 : 0;
            else if (diff >= 0.3) frozen_ticks_ = 0;     // frames differ -> clearly not frozen
            if (frozen_ticks_ >= 5) freeze_score = 0.0;
        }
        prev_sample_ = std::move(sample);

        // baseline of the clean scene's natural texture
        if (frames_ < 30) {
            baseline_ = frames_ == 0 ? noise_est : 0.8 * baseline_ + 0.2 * noise_est;
        } else if (noise_est < baseline_ * 1.5) {
            baseline_ = 0.99 * baseline_ + 0.01 * noise_est;
        }
        frames_++;

        const double noise_score = std::exp(-std::max(0.0, noise_est - baseline_) / 10.0);
        const double quality = std::clamp(freeze_score * noise_score, 0.0, 1.0);
        config().blackboard->set(name() + "_quality_score", quality);
        cout << name() << " quality: " << quality << " (noise " << noise_est
             << ", base " << baseline_ << ", frozen " << frozen_ticks_ << ")" << endl;
        return BT::NodeStatus::SUCCESS;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    sensor_msgs::msg::Image::SharedPtr last_msg_;
    std::mutex mtx_;
    std::vector<uint8_t> prev_sample_;
    int frozen_ticks_ = 0;
    double baseline_ = 0.0;
    int frames_ = 0;
};
