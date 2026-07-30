#pragma once
#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <algorithm>
#include <cmath>
#include <mutex>
using namespace std;

// Estimates pointcloud quality (0~1) from the actual data, independent of the
// timeout/FTTI check: gaussian range noise raises the neighbor range difference,
// dropout lowers the point count. quality = noise_score * count_score.
class LidarQualityChecker : public BT::SyncActionNode
{
public:
    LidarQualityChecker(const std::string& name, const BT::NodeConfig& conf,
                        const BT::RosNodeParams& params)
                        : SyncActionNode(name, conf)
    {
        node_ = params.nh.lock();
        std::string topic;
        if (getInput("topic_name", topic) && !topic.empty()) {
            sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
                topic, rclcpp::SensorDataQoS(),
                [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                    std::lock_guard<std::mutex> lk(mtx_);
                    last_msg_ = msg;
                });
        }
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("topic_name"),
        };
    }

    BT::NodeStatus tick() override
    {
        sensor_msgs::msg::PointCloud2::SharedPtr msg;
        {
            std::lock_guard<std::mutex> lk(mtx_);
            msg = last_msg_;
        }
        if (!msg) {
            // no data yet — leave quality at 1.0, the timeout checker owns absence
            config().blackboard->set(name() + "_quality_score", 1.0);
            return BT::NodeStatus::SUCCESS;
        }

        int ox = -1, oy = -1, oz = -1;
        for (const auto& f : msg->fields) {
            if (f.datatype != sensor_msgs::msg::PointField::FLOAT32) continue;
            if (f.name == "x") ox = f.offset;
            else if (f.name == "y") oy = f.offset;
            else if (f.name == "z") oz = f.offset;
        }
        const size_t n = static_cast<size_t>(msg->width) * msg->height;
        if (ox < 0 || oy < 0 || oz < 0 || n < 100) {
            config().blackboard->set(name() + "_quality_score", 1.0);
            return BT::NodeStatus::SUCCESS;
        }

        // sigma estimate from consecutive-point range differences.
        // For i.i.d. gaussian range noise, E|r1 - r2| = 2*sigma/sqrt(pi).
        const size_t stride = std::max<size_t>(1, n / 4000);
        double acc = 0.0;
        size_t cnt = 0;
        auto range_at = [&](size_t i) {
            const uint8_t* p = &msg->data[i * msg->point_step];
            float x, y, z;
            memcpy(&x, p + ox, 4); memcpy(&y, p + oy, 4); memcpy(&z, p + oz, 4);
            return std::sqrt(x * x + y * y + z * z);
        };
        for (size_t i = 0; i + 1 < n; i += stride) {
            const double r1 = range_at(i), r2 = range_at(i + 1);
            if (!std::isfinite(r1) || !std::isfinite(r2) || r1 < 0.5 || r2 < 0.5) continue;
            const double dr = std::fabs(r1 - r2);
            if (dr < 1.0) { acc += dr; cnt++; }   // reject real depth edges
        }
        const double sigma_est = cnt > 20 ? (acc / cnt) * 0.886 : 0.0;  // sqrt(pi)/2

        // slow baseline of the clean scene so structural roughness isn't
        // counted as noise; refuses to follow sudden (fault-like) jumps
        if (frames_ < 30) {
            baseline_ = frames_ == 0 ? sigma_est : 0.8 * baseline_ + 0.2 * sigma_est;
        } else if (sigma_est < baseline_ * 1.5) {
            baseline_ = 0.99 * baseline_ + 0.01 * sigma_est;
        }
        frames_++;

        const double noise_score = std::exp(-std::max(0.0, sigma_est - baseline_) / 0.1);

        if (frames_ < 30) {
            count_baseline_ = count_baseline_ == 0.0 ? double(n)
                                                     : 0.8 * count_baseline_ + 0.2 * n;
        } else if (n > count_baseline_ * 0.7) {
            count_baseline_ = 0.99 * count_baseline_ + 0.01 * n;
        }
        const double count_score =
            count_baseline_ > 0 ? std::clamp(n / count_baseline_, 0.0, 1.0) : 1.0;

        const double quality = std::clamp(noise_score * count_score, 0.0, 1.0);
        config().blackboard->set(name() + "_quality_score", quality);
        cout << name() << " quality: " << quality << " (sigma " << sigma_est
             << "m, base " << baseline_ << "m, pts " << n << ")" << endl;
        return BT::NodeStatus::SUCCESS;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    sensor_msgs::msg::PointCloud2::SharedPtr last_msg_;
    std::mutex mtx_;
    double baseline_ = 0.0;
    double count_baseline_ = 0.0;
    int frames_ = 0;
};
