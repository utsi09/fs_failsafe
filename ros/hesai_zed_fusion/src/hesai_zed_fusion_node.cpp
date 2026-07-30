#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <map>
#include <sstream>
#include <iomanip>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "cv_bridge/cv_bridge.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

#include "hesai_zed_fusion/msg/obstacle_info.hpp"
#include "hesai_zed_fusion/msg/obstacle_array.hpp"

#include <Eigen/Dense>
#include <cuda_runtime_api.h>
#include <NvInfer.h>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;
using namespace std::chrono;

const int INPUT_W = 640;
const int INPUT_H = 640;
const int NUM_CLASSES = 80;
const int NUM_MASKS = 32;
const int OUTPUT_CHANNELS = 4 + NUM_CLASSES + NUM_MASKS;
const int OUTPUT_GRID = 8400;

struct Detection {
    int class_id;
    float conf;
    cv::Rect box;
    cv::Mat mask;
};

class Logger : public nvinfer1::ILogger {
    void log(Severity severity, const char* msg) noexcept override {
        // if (severity <= Severity::kWARNING)
        //     std::cout << "[TRT] " << msg << std::endl;
    }
} gLogger;

class YoloDetector {
public:
    YoloDetector(const string& engine_path, float conf_thres, float iou_thres)
        : conf_thres_(conf_thres), iou_thres_(iou_thres)
    {
        std::ifstream file(engine_path, ios::binary);
        if (!file.good()) {
            std::cerr << "Error: Engine file not found: " << engine_path << std::endl;
            return;
        }
        file.seekg(0, file.end);
        size_t size = file.tellg();
        file.seekg(0, file.beg);
        std::vector<char> engine_data(size);
        file.read(engine_data.data(), size);
        file.close();

        runtime_ = nvinfer1::createInferRuntime(gLogger);
        engine_ = runtime_->deserializeCudaEngine(engine_data.data(), size);
        context_ = engine_->createExecutionContext();

        cudaStreamCreate(&stream_);

        cudaMalloc(&buffers_[0], 1 * 3 * INPUT_H * INPUT_W * sizeof(float));
        cudaMalloc(&buffers_[1], 1 * OUTPUT_CHANNELS * OUTPUT_GRID * sizeof(float));
        cudaMalloc(&buffers_[2], 1 * NUM_MASKS * 160 * 160 * sizeof(float));

        context_->setInputShape("images", nvinfer1::Dims4{1, 3, INPUT_H, INPUT_W});
        context_->setTensorAddress("images", buffers_[0]);
        context_->setTensorAddress("output0", buffers_[1]);
        context_->setTensorAddress("output1", buffers_[2]);

        std::cout << "TensorRT Engine Loaded (Batch 1, Segmentation)!" << std::endl;
    }

    ~YoloDetector() {
        cudaFree(buffers_[0]);
        cudaFree(buffers_[1]);
        cudaFree(buffers_[2]);
        cudaStreamDestroy(stream_);
        delete context_;
        delete engine_;
        delete runtime_;
    }

    vector<Detection> detect(const cv::Mat& img) {
        vector<Detection> results;
        if (!context_ || img.empty()) return results;

        cv::Mat blob = cv::dnn::blobFromImage(img, 1.0 / 255.0,
            cv::Size(INPUT_W, INPUT_H), cv::Scalar(), true, false);

        cudaMemcpyAsync(buffers_[0], blob.ptr<float>(),
            blob.total() * sizeof(float), cudaMemcpyHostToDevice, stream_);
        context_->enqueueV3(stream_);

        std::vector<float> output0(OUTPUT_CHANNELS * OUTPUT_GRID);
        std::vector<float> output1(NUM_MASKS * 160 * 160);

        cudaMemcpyAsync(output0.data(), buffers_[1],
            output0.size() * sizeof(float), cudaMemcpyDeviceToHost, stream_);
        cudaMemcpyAsync(output1.data(), buffers_[2],
            output1.size() * sizeof(float), cudaMemcpyDeviceToHost, stream_);
        cudaStreamSynchronize(stream_);

        float x_scale = (float)img.cols / INPUT_W;
        float y_scale = (float)img.rows / INPUT_H;

        std::vector<int> class_ids;
        std::vector<float> confidences;
        std::vector<cv::Rect> boxes;
        std::vector<std::vector<float>> mask_coeffs;

        cv::Mat output0_mat(OUTPUT_CHANNELS, OUTPUT_GRID, CV_32F, output0.data());

        for (int i = 0; i < OUTPUT_GRID; ++i) {
            float max_conf = 0.0f;
            int max_class_id = -1;

            for (int c = 0; c < NUM_CLASSES; ++c) {
                float score = output0_mat.at<float>(4 + c, i);
                if (score > max_conf) {
                    max_conf = score;
                    max_class_id = c;
                }
            }

            if (max_conf > conf_thres_) {
                float cx = output0_mat.at<float>(0, i);
                float cy = output0_mat.at<float>(1, i);
                float w = output0_mat.at<float>(2, i);
                float h = output0_mat.at<float>(3, i);

                int left = int((cx - 0.5f * w) * x_scale);
                int top = int((cy - 0.5f * h) * y_scale);
                int width = int(w * x_scale);
                int height = int(h * y_scale);

                std::vector<float> coeffs(NUM_MASKS);
                for (int m = 0; m < NUM_MASKS; ++m)
                    coeffs[m] = output0_mat.at<float>(4 + NUM_CLASSES + m, i);

                boxes.push_back(cv::Rect(left, top, width, height));
                class_ids.push_back(max_class_id);
                confidences.push_back(max_conf);
                mask_coeffs.push_back(coeffs);
            }
        }

        std::vector<int> indices;
        cv::dnn::NMSBoxes(boxes, confidences, conf_thres_, iou_thres_, indices);

        cv::Mat proto(NUM_MASKS, 160 * 160, CV_32F, output1.data());

        for (int idx : indices) {
            Detection det;
            det.class_id = class_ids[idx];
            det.conf = confidences[idx];
            det.box = boxes[idx] & cv::Rect(0, 0, img.cols, img.rows);

            if (det.box.area() > 0) {
                cv::Mat coeff_mat(1, NUM_MASKS, CV_32F, mask_coeffs[idx].data());
                cv::Mat mask_logits = coeff_mat * proto;
                cv::Mat mask_proto = mask_logits.reshape(1, 160);

                cv::exp(-mask_proto, mask_proto);
                mask_proto = 1.0 / (1.0 + mask_proto);

                cv::Mat mask_resized;
                cv::resize(mask_proto, mask_resized, cv::Size(INPUT_W, INPUT_H));
                cv::resize(mask_resized, mask_resized, img.size());

                cv::Mat final_mask = mask_resized(det.box) > 0.5;
                final_mask.convertTo(det.mask, CV_8UC1, 255);

                results.push_back(det);
            }
        }
        return results;
    }

private:
    float conf_thres_;
    float iou_thres_;
    cudaStream_t stream_;
    nvinfer1::IRuntime* runtime_ = nullptr;
    nvinfer1::ICudaEngine* engine_ = nullptr;
    nvinfer1::IExecutionContext* context_ = nullptr;
    void* buffers_[3];
};

class HesaiZedFusionNode : public rclcpp::Node {
public:
    HesaiZedFusionNode()
        : Node("hesai_zed_fusion"),
          tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
          tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
    {
        declare_parameter("model_path", "yolo26x-seg.engine");
        declare_parameter("conf", 0.3);
        declare_parameter("iou", 0.5);
        declare_parameter("image_topic", "/zed/zed_node/rgb/color/rect/image/compressed");
        declare_parameter("lidar_topic", "/lidar_points");
        declare_parameter("camera_info_topic", "/zed/zed_node/rgb/color/rect/camera_info");
        declare_parameter("lidar_frame", "hesai_lidar");
        declare_parameter("base_frame", "base_link");
        declare_parameter("camera_frame", "");
        declare_parameter("mask_scale", 0.8);

        std::string model_path = get_parameter("model_path").as_string();
        float conf = get_parameter("conf").as_double();
        float iou = get_parameter("iou").as_double();
        std::string image_topic = get_parameter("image_topic").as_string();
        std::string lidar_topic = get_parameter("lidar_topic").as_string();
        std::string camera_info_topic = get_parameter("camera_info_topic").as_string();
        lidar_frame_ = get_parameter("lidar_frame").as_string();
        base_frame_ = get_parameter("base_frame").as_string();
        camera_frame_ = get_parameter("camera_frame").as_string();
        mask_scale_ = get_parameter("mask_scale").as_double();

        detector_ = std::make_shared<YoloDetector>(model_path, conf, iou);

        pub_obstacles_ = create_publisher<hesai_zed_fusion::msg::ObstacleArray>(
            "hesai_zed/obstacles", 10);
        pub_overlay_ = create_publisher<sensor_msgs::msg::Image>(
            "hesai_zed/overlay", 10);
        pub_obj_pc_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            "hesai_zed/object_pointcloud", 10);

        sub_camera_info_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            camera_info_topic, rclcpp::SensorDataQoS(),
            std::bind(&HesaiZedFusionNode::camera_info_callback, this, std::placeholders::_1));

        rmw_qos_profile_t sensor_qos = rmw_qos_profile_sensor_data;
        sensor_qos.depth = 10;
        sub_image_.subscribe(this, image_topic, sensor_qos);
        sub_lidar_.subscribe(this, lidar_topic, sensor_qos);

        sync_ = std::make_shared<Sync>(SyncPolicy(10), sub_image_, sub_lidar_);
        sync_->registerCallback(std::bind(&HesaiZedFusionNode::sync_callback, this,
            std::placeholders::_1, std::placeholders::_2));

        class_names_[0] = "person";
        class_names_[1] = "bicycle";
        class_names_[2] = "car";
        class_names_[3] = "motorcycle";
        class_names_[5] = "bus";
        class_names_[7] = "truck";
        target_classes_ = {0, 1, 2, 3, 5, 7};

        RCLCPP_INFO(get_logger(), "Hesai-ZED Fusion Node initialized");
    }

private:
    void camera_info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg) {
        if (!camera_info_received_) {
            K_ << msg->k[0], msg->k[1], msg->k[2],
                  msg->k[3], msg->k[4], msg->k[5],
                  msg->k[6], msg->k[7], msg->k[8];

            if (camera_frame_.empty()) {
                camera_frame_ = msg->header.frame_id;
            }
            camera_info_received_ = true;
        }
    }

    Eigen::Matrix4f tf_to_eigen(const geometry_msgs::msg::TransformStamped& tf) {
        Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
        Eigen::Quaternionf q(
            tf.transform.rotation.w,
            tf.transform.rotation.x,
            tf.transform.rotation.y,
            tf.transform.rotation.z);
        T.block<3, 3>(0, 0) = q.toRotationMatrix();
        T(0, 3) = tf.transform.translation.x;
        T(1, 3) = tf.transform.translation.y;
        T(2, 3) = tf.transform.translation.z;
        return T;
    }

    bool cache_transforms() {
        if (transforms_cached_) return true;
        if (camera_frame_.empty()) return false;

        try {
            auto tf_cam = tf_buffer_->lookupTransform(
                camera_frame_, lidar_frame_, tf2::TimePointZero);
            T_cam_lidar_ = tf_to_eigen(tf_cam);

            auto tf_base = tf_buffer_->lookupTransform(
                base_frame_, lidar_frame_, tf2::TimePointZero);
            T_base_lidar_ = tf_to_eigen(tf_base);

            transforms_cached_ = true;
            return true;
        } catch (tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "TF not ready: %s", ex.what());
            return false;
        }
    }

    void sync_callback(
        const sensor_msgs::msg::CompressedImage::ConstSharedPtr& img_msg,
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr& lidar_msg)
    {
        if (!camera_info_received_ || !cache_transforms()) return;

        cv::Mat raw_data(1, img_msg->data.size(), CV_8UC1, (void*)img_msg->data.data());
        cv::Mat cv_img = cv::imdecode(raw_data, cv::IMREAD_COLOR);
        if (cv_img.empty()) return;

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*lidar_msg, *cloud);
        if (cloud->empty()) return;

        auto results = detector_->detect(cv_img);

        std::vector<cv::Point> valid_uvs;
        std::vector<int> valid_indices;
        std::vector<bool> is_object_point(cloud->points.size(), false);

        for (size_t k = 0; k < cloud->points.size(); k++) {
            const auto& pt = cloud->points[k];
            Eigen::Vector4f p_lidar(pt.x, pt.y, pt.z, 1.0f);
            Eigen::Vector4f p_cam = T_cam_lidar_ * p_lidar;

            if (p_cam(2) <= 0) continue;

            Eigen::Vector3f proj = K_ * p_cam.head<3>();
            if (proj(2) == 0) continue;

            int u = static_cast<int>(proj(0) / proj(2));
            int v = static_cast<int>(proj(1) / proj(2));

            if (u >= 0 && u < cv_img.cols && v >= 0 && v < cv_img.rows) {
                valid_uvs.emplace_back(u, v);
                valid_indices.push_back(k);
            }
        }

        hesai_zed_fusion::msg::ObstacleArray obs_array;
        obs_array.header = lidar_msg->header;
        obs_array.header.frame_id = base_frame_;

        for (auto& det : results) {
            if (det.box.area() <= 0) continue;

            std::string label = class_names_.count(det.class_id)
                ? class_names_[det.class_id] : "class_" + std::to_string(det.class_id);

            cv::rectangle(cv_img, det.box, cv::Scalar(0, 255, 0), 2);
            cv::putText(cv_img, label, det.box.tl(), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);

            if (!det.mask.empty()) {
                cv::Rect valid_box = det.box & cv::Rect(0, 0, cv_img.cols, cv_img.rows);
                if (valid_box.area() > 0) {
                    cv::Mat colored_mask;
                    cv::cvtColor(det.mask, colored_mask, cv::COLOR_GRAY2BGR);
                    colored_mask.setTo(cv::Scalar(0, 0, 255), det.mask > 0);
                    cv::Mat roi = cv_img(valid_box);
                    cv::Mat mask_roi = colored_mask(cv::Rect(0, 0, valid_box.width, valid_box.height));
                    cv::addWeighted(roi, 1.0, mask_roi, 0.4, 0.0, roi);
                }
            }

            bool is_target = false;
            for (int t : target_classes_) {
                if (det.class_id == t) { is_target = true; break; }
            }
            if (!is_target || det.mask.empty()) continue;

            cv::Mat check_mask;
            int new_w = static_cast<int>(det.mask.cols * mask_scale_);
            int new_h = static_cast<int>(det.mask.rows * mask_scale_);

            if (new_w > 0 && new_h > 0) {
                check_mask = cv::Mat::zeros(det.mask.size(), det.mask.type());
                cv::Mat resized;
                cv::resize(det.mask, resized, cv::Size(new_w, new_h), 0, 0, cv::INTER_NEAREST);
                int ox = (det.mask.cols - new_w) / 2;
                int oy = (det.mask.rows - new_h) / 2;
                resized.copyTo(check_mask(cv::Rect(ox, oy, new_w, new_h)));
            } else {
                check_mask = det.mask;
            }

            float min_dist = 9999.0f;
            Eigen::Vector3f closest_point(0, 0, 0);
            int point_count = 0;

            for (size_t k = 0; k < valid_uvs.size(); ++k) {
                const cv::Point& pt = valid_uvs[k];
                if (!det.box.contains(pt)) continue;

                int mx = pt.x - det.box.x;
                int my = pt.y - det.box.y;

                if (mx >= 0 && mx < check_mask.cols && my >= 0 && my < check_mask.rows) {
                    if (check_mask.at<uint8_t>(my, mx) > 0) {
                        int idx = valid_indices[k];
                        is_object_point[idx] = true;
                        const auto& p3d = cloud->points[idx];
                        Eigen::Vector3f p(p3d.x, p3d.y, p3d.z);
                        float dist = p.norm();
                        if (dist < min_dist) {
                            min_dist = dist;
                            closest_point = p;
                        }
                        point_count++;
                    }
                }
            }

            if (point_count > 0) {
                Eigen::Vector4f p_lidar(closest_point(0), closest_point(1), closest_point(2), 1.0f);
                Eigen::Vector4f p_base = T_base_lidar_ * p_lidar;

                hesai_zed_fusion::msg::ObstacleInfo obs;
                obs.label = label;
                obs.class_id = det.class_id;
                obs.confidence = det.conf;
                obs.rel_x = p_base(0);
                obs.rel_y = p_base(1);
                obs.rel_z = p_base(2);
                obs.distance = min_dist;
                obs_array.obstacles.push_back(obs);

                std::stringstream ss;
                ss << std::fixed << std::setprecision(1) << min_dist;
                cv::putText(cv_img, label + " " + ss.str() + "m", cv::Point(det.box.x, det.box.y - 10), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
            }
        }

        pub_obstacles_->publish(obs_array);
        auto overlay_msg = cv_bridge::CvImage(img_msg->header, "bgr8", cv_img).toImageMsg();
        pub_overlay_->publish(*overlay_msg);

        pcl::PointCloud<pcl::PointXYZ> obj_cloud;
        for (size_t k = 0; k < cloud->points.size(); k++) {
            if (is_object_point[k]) obj_cloud.push_back(cloud->points[k]);
        }
        if (!obj_cloud.empty()) {
            sensor_msgs::msg::PointCloud2 pc_msg;
            pcl::toROSMsg(obj_cloud, pc_msg);
            pc_msg.header = lidar_msg->header;
            pub_obj_pc_->publish(pc_msg);
        }
    }

    using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::CompressedImage, sensor_msgs::msg::PointCloud2>;
    using Sync = message_filters::Synchronizer<SyncPolicy>;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<YoloDetector> detector_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_camera_info_;
    message_filters::Subscriber<sensor_msgs::msg::CompressedImage> sub_image_;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub_lidar_;
    std::shared_ptr<Sync> sync_;
    rclcpp::Publisher<hesai_zed_fusion::msg::ObstacleArray>::SharedPtr pub_obstacles_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_overlay_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_obj_pc_;

    Eigen::Matrix3f K_;
    bool camera_info_received_ = false;
    Eigen::Matrix4f T_cam_lidar_;
    Eigen::Matrix4f T_base_lidar_;
    bool transforms_cached_ = false;
    std::string lidar_frame_;
    std::string base_frame_;
    std::string camera_frame_;
    float mask_scale_;
    std::map<int, std::string> class_names_;
    std::vector<int> target_classes_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HesaiZedFusionNode>());
    rclcpp::shutdown();
    return 0;
}