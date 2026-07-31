// ============================================================================
// [이 파일의 역할]
// ★ 실차(real vehicle) 포팅용 모듈이다. CARLA 시뮬레이션이 아니라, 실제 차량에
//   달린 Hesai 라이다 + ZED 카메라를 융합(fusion)하는 ROS2 노드.
//   (시뮬레이션에서는 CARLA 브리지가 장애물 정보를 주지만, 실차에서는 이 노드가
//    그 역할을 대신한다. Python 버전 fusion_node.py와 같은 일을 하는
//    C++ / TensorRT 고성능 버전.)
//
// 하는 일 (한 프레임 기준):
//   1) 카메라 이미지에서 YOLO 세그멘테이션으로 물체(사람/차/트럭...)를 찾고
//   2) 라이다 3D 점들을 카메라 이미지 평면에 투영(projection)해서
//   3) "물체 마스크 안에 찍힌" 라이다 점만 골라낸 뒤
//   4) 그중 가장 가까운 점의 3D 좌표를 차량 기준(base_link)으로 변환해
//   5) hesai_zed/obstacles (ObstacleArray) 토픽으로 발행한다.
//      → 이 토픽을 cdr_monitor/mrm_supervisor 쪽에서 장애물 거리(TTC 계산)로 쓴다.
//
// 입력 토픽: 카메라 압축 이미지, 라이다 PointCloud2, 카메라 내부 파라미터(CameraInfo)
// 출력 토픽: hesai_zed/obstacles(장애물 목록), hesai_zed/overlay(디버그 시각화 이미지),
//           hesai_zed/object_pointcloud(물체에 속한 라이다 점만 모은 클라우드)
// ============================================================================

// ---- 표준 C++ 라이브러리 ----
#include <fstream>      // 파일 읽기 (TensorRT 엔진 파일 로드용)
#include <iostream>
#include <vector>
#include <string>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <map>
#include <sstream>
#include <iomanip>

// ---- ROS2 관련 ----
#include "rclcpp/rclcpp.hpp"                       // ROS2 C++ 기본 (Node, 발행/구독)
#include "sensor_msgs/msg/compressed_image.hpp"    // 압축(JPEG) 이미지 메시지
#include "sensor_msgs/msg/image.hpp"               // 비압축 이미지 메시지 (오버레이 출력용)
#include "sensor_msgs/msg/point_cloud2.hpp"        // 라이다 포인트클라우드 메시지
#include "sensor_msgs/msg/camera_info.hpp"         // 카메라 내부 파라미터(K 행렬) 메시지
#include "tf2_ros/buffer.h"                        // TF(좌표계 변환) 저장소
#include "tf2_ros/transform_listener.h"            // TF 변환을 자동으로 수신
#include "cv_bridge/cv_bridge.h"                   // ROS 이미지 <-> OpenCV 이미지 변환
#include "pcl_conversions/pcl_conversions.h"       // ROS PointCloud2 <-> PCL 클라우드 변환
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "message_filters/subscriber.h"            // 여러 토픽을 시간 맞춰 묶는 도구
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"  // "대략 비슷한 시각" 짝 맞추기 정책

// ---- 이 패키지의 커스텀 메시지 (장애물 1개 / 장애물 배열) ----
#include "hesai_zed_fusion/msg/obstacle_info.hpp"
#include "hesai_zed_fusion/msg/obstacle_array.hpp"

// ---- 수학/GPU/딥러닝 ----
#include <Eigen/Dense>          // 행렬 연산 라이브러리 (좌표 변환 계산)
#include <cuda_runtime_api.h>   // NVIDIA GPU 메모리 관리 (cudaMalloc 등)
#include <NvInfer.h>            // TensorRT: GPU에서 딥러닝 모델을 빠르게 돌리는 엔진
#include <opencv2/opencv.hpp>   // 영상 처리 (디코딩, 리사이즈, NMS, 그리기)

using namespace std;            // std:: 접두사 생략
using namespace cv;             // cv:: 접두사 생략
using namespace std::chrono;

// ---- YOLO 모델 입출력 크기 상수 (모델 구조에 따라 고정된 값) ----
const int INPUT_W = 640;        // YOLO 입력 이미지 가로
const int INPUT_H = 640;        // YOLO 입력 이미지 세로
const int NUM_CLASSES = 80;     // COCO 데이터셋 클래스 수 (사람, 차, ...)
const int NUM_MASKS = 32;       // 세그멘테이션 마스크 계수 개수
const int OUTPUT_CHANNELS = 4 + NUM_CLASSES + NUM_MASKS;  // 박스(4) + 클래스점수(80) + 마스크계수(32)
const int OUTPUT_GRID = 8400;   // YOLO가 한 번에 내놓는 후보 박스 개수

// 탐지 결과 1개를 담는 구조체: 클래스 번호, 신뢰도, 박스 위치, 물체 모양 마스크
struct Detection {
    int class_id;
    float conf;
    cv::Rect box;
    cv::Mat mask;
};

// TensorRT가 내부 로그를 출력할 때 쓰는 클래스. 여기서는 로그를 전부 무시(조용히).
class Logger : public nvinfer1::ILogger {
    void log(Severity severity, const char* msg) noexcept override {
        // if (severity <= Severity::kWARNING)
        //     std::cout << "[TRT] " << msg << std::endl;
    }
} gLogger;

// ============================================================================
// YoloDetector: TensorRT로 YOLO 세그멘테이션 추론을 담당하는 클래스.
//   ROS와는 무관한 순수 "이미지 → 탐지 결과" 변환기.
// ============================================================================
class YoloDetector {
public:
    // 생성자: 미리 변환해 둔 TensorRT 엔진 파일(.engine)을 읽어 GPU에 올린다.
    // conf_thres = 이 신뢰도보다 낮은 탐지는 버림, iou_thres = 겹친 박스 제거 기준
    YoloDetector(const string& engine_path, float conf_thres, float iou_thres)
        : conf_thres_(conf_thres), iou_thres_(iou_thres)
    {
        // 엔진 파일을 통째로 바이너리로 읽는다
        std::ifstream file(engine_path, ios::binary);
        if (!file.good()) {
            std::cerr << "Error: Engine file not found: " << engine_path << std::endl;
            return;
        }
        file.seekg(0, file.end);
        size_t size = file.tellg();       // 파일 끝으로 이동해서 크기 측정
        file.seekg(0, file.beg);
        std::vector<char> engine_data(size);
        file.read(engine_data.data(), size);
        file.close();

        // 읽은 바이트를 GPU에서 실행 가능한 엔진 객체로 복원(deserialize)
        runtime_ = nvinfer1::createInferRuntime(gLogger);
        engine_ = runtime_->deserializeCudaEngine(engine_data.data(), size);
        context_ = engine_->createExecutionContext();

        cudaStreamCreate(&stream_);       // GPU 작업 대기열(스트림) 생성

        // GPU 메모리 할당: [0]=입력 이미지, [1]=박스 출력, [2]=마스크 프로토타입 출력
        cudaMalloc(&buffers_[0], 1 * 3 * INPUT_H * INPUT_W * sizeof(float));
        cudaMalloc(&buffers_[1], 1 * OUTPUT_CHANNELS * OUTPUT_GRID * sizeof(float));
        cudaMalloc(&buffers_[2], 1 * NUM_MASKS * 160 * 160 * sizeof(float));

        // 엔진에 입출력 텐서의 크기와 GPU 주소를 알려준다
        context_->setInputShape("images", nvinfer1::Dims4{1, 3, INPUT_H, INPUT_W});
        context_->setTensorAddress("images", buffers_[0]);
        context_->setTensorAddress("output0", buffers_[1]);
        context_->setTensorAddress("output1", buffers_[2]);

        std::cout << "TensorRT Engine Loaded (Batch 1, Segmentation)!" << std::endl;
    }

    // 소멸자: 객체가 사라질 때 GPU 메모리와 엔진 객체를 정리 (C++은 수동 해제 필요)
    ~YoloDetector() {
        cudaFree(buffers_[0]);
        cudaFree(buffers_[1]);
        cudaFree(buffers_[2]);
        cudaStreamDestroy(stream_);
        delete context_;
        delete engine_;
        delete runtime_;
    }

    // 핵심 함수: 이미지 1장을 받아 탐지 결과 목록을 돌려준다.
    // 순서: 전처리 → GPU 복사 → 추론 → 결과 회수 → 후보 필터링 → NMS → 마스크 생성
    vector<Detection> detect(const cv::Mat& img) {
        vector<Detection> results;
        if (!context_ || img.empty()) return results;

        // 전처리: 픽셀값을 0~1로 정규화하고 640x640으로 리사이즈, BGR→RGB
        cv::Mat blob = cv::dnn::blobFromImage(img, 1.0 / 255.0,
            cv::Size(INPUT_W, INPUT_H), cv::Scalar(), true, false);

        // CPU 메모리 → GPU 메모리로 입력 복사 후 추론 실행
        cudaMemcpyAsync(buffers_[0], blob.ptr<float>(),
            blob.total() * sizeof(float), cudaMemcpyHostToDevice, stream_);
        context_->enqueueV3(stream_);     // GPU에서 YOLO 추론 실행 (비동기)

        std::vector<float> output0(OUTPUT_CHANNELS * OUTPUT_GRID);
        std::vector<float> output1(NUM_MASKS * 160 * 160);

        // 추론 결과를 GPU → CPU로 다시 복사
        cudaMemcpyAsync(output0.data(), buffers_[1],
            output0.size() * sizeof(float), cudaMemcpyDeviceToHost, stream_);
        cudaMemcpyAsync(output1.data(), buffers_[2],
            output1.size() * sizeof(float), cudaMemcpyDeviceToHost, stream_);
        cudaStreamSynchronize(stream_);   // 위의 비동기 작업들이 다 끝날 때까지 대기

        // YOLO 좌표(640x640 기준)를 원본 이미지 크기로 되돌릴 배율
        float x_scale = (float)img.cols / INPUT_W;
        float y_scale = (float)img.rows / INPUT_H;

        std::vector<int> class_ids;
        std::vector<float> confidences;
        std::vector<cv::Rect> boxes;
        std::vector<std::vector<float>> mask_coeffs;

        // 출력 배열을 (116행 x 8400열) 행렬로 해석: 열 하나가 후보 박스 하나
        cv::Mat output0_mat(OUTPUT_CHANNELS, OUTPUT_GRID, CV_32F, output0.data());

        // 8400개 후보를 전부 훑으며 신뢰도가 기준 이상인 것만 수집
        for (int i = 0; i < OUTPUT_GRID; ++i) {
            float max_conf = 0.0f;
            int max_class_id = -1;

            // 80개 클래스 중 점수가 가장 높은 클래스를 찾는다
            for (int c = 0; c < NUM_CLASSES; ++c) {
                float score = output0_mat.at<float>(4 + c, i);
                if (score > max_conf) {
                    max_conf = score;
                    max_class_id = c;
                }
            }

            if (max_conf > conf_thres_) {
                // 앞 4개 값 = 박스 중심(cx,cy)과 크기(w,h)
                float cx = output0_mat.at<float>(0, i);
                float cy = output0_mat.at<float>(1, i);
                float w = output0_mat.at<float>(2, i);
                float h = output0_mat.at<float>(3, i);

                // 중심+크기 → 좌상단+크기 형태로 바꾸고 원본 크기로 스케일
                int left = int((cx - 0.5f * w) * x_scale);
                int top = int((cy - 0.5f * h) * y_scale);
                int width = int(w * x_scale);
                int height = int(h * y_scale);

                // 마스크 계수 32개도 함께 보관 (나중에 마스크 복원에 사용)
                std::vector<float> coeffs(NUM_MASKS);
                for (int m = 0; m < NUM_MASKS; ++m)
                    coeffs[m] = output0_mat.at<float>(4 + NUM_CLASSES + m, i);

                boxes.push_back(cv::Rect(left, top, width, height));
                class_ids.push_back(max_class_id);
                confidences.push_back(max_conf);
                mask_coeffs.push_back(coeffs);
            }
        }

        // NMS(비최대 억제): 같은 물체에 겹쳐 잡힌 박스들 중 최고 점수 하나만 남김
        std::vector<int> indices;
        cv::dnn::NMSBoxes(boxes, confidences, conf_thres_, iou_thres_, indices);

        // 마스크 프로토타입: 32장의 160x160 기본 마스크 (계수와 곱해 실제 마스크를 만듦)
        cv::Mat proto(NUM_MASKS, 160 * 160, CV_32F, output1.data());

        for (int idx : indices) {
            Detection det;
            det.class_id = class_ids[idx];
            det.conf = confidences[idx];
            det.box = boxes[idx] & cv::Rect(0, 0, img.cols, img.rows);  // &: 이미지 영역 밖으로 나간 부분 잘라냄

            if (det.box.area() > 0) {
                // 마스크 복원: (1x32 계수) x (32x25600 프로토타입) = 물체 모양 점수맵
                cv::Mat coeff_mat(1, NUM_MASKS, CV_32F, mask_coeffs[idx].data());
                cv::Mat mask_logits = coeff_mat * proto;
                cv::Mat mask_proto = mask_logits.reshape(1, 160);   // 1행 → 160x160 이미지 모양으로

                // 시그모이드 함수 1/(1+e^-x): 점수를 0~1 확률로 변환
                cv::exp(-mask_proto, mask_proto);
                mask_proto = 1.0 / (1.0 + mask_proto);

                // 160x160 마스크를 원본 이미지 크기까지 키움
                cv::Mat mask_resized;
                cv::resize(mask_proto, mask_resized, cv::Size(INPUT_W, INPUT_H));
                cv::resize(mask_resized, mask_resized, img.size());

                // 박스 영역만 잘라 0.5 초과 픽셀 = 물체(255), 나머지 = 배경(0)
                cv::Mat final_mask = mask_resized(det.box) > 0.5;
                final_mask.convertTo(det.mask, CV_8UC1, 255);

                results.push_back(det);
            }
        }
        return results;
    }

private:
    float conf_thres_;                                  // 신뢰도 하한
    float iou_thres_;                                   // NMS 겹침 기준
    cudaStream_t stream_;                               // GPU 작업 스트림
    nvinfer1::IRuntime* runtime_ = nullptr;             // TensorRT 런타임 (raw 포인터: 직접 delete 필요)
    nvinfer1::ICudaEngine* engine_ = nullptr;           // 로드된 모델 엔진
    nvinfer1::IExecutionContext* context_ = nullptr;    // 추론 실행 컨텍스트
    void* buffers_[3];                                  // GPU 메모리 주소 3개 (입력1 + 출력2)
};

// ============================================================================
// HesaiZedFusionNode: ROS2 노드 본체. 토픽 구독/발행과 융합 로직을 담당.
//   rclcpp::Node를 상속받으면 ROS2 노드가 된다 (파라미터/토픽/로그 기능을 물려받음).
// ============================================================================
class HesaiZedFusionNode : public rclcpp::Node {
public:
    // 생성자: 노드 이름 "hesai_zed_fusion"으로 등록하고 TF 수신기를 켠다
    HesaiZedFusionNode()
        : Node("hesai_zed_fusion"),
          tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
          tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
    {
        // 파라미터 선언: 런치파일/명령행에서 바꿀 수 있는 설정값 (기본값 포함)
        declare_parameter("model_path", "yolo26x-seg.engine");   // TensorRT 엔진 파일 경로
        declare_parameter("conf", 0.3);                          // YOLO 신뢰도 문턱
        declare_parameter("iou", 0.5);                           // NMS 겹침 문턱
        declare_parameter("image_topic", "/zed/zed_node/rgb/color/rect/image/compressed");
        declare_parameter("lidar_topic", "/lidar_points");
        declare_parameter("camera_info_topic", "/zed/zed_node/rgb/color/rect/camera_info");
        declare_parameter("lidar_frame", "hesai_lidar");         // 라이다 좌표계 이름
        declare_parameter("base_frame", "base_link");            // 차량 기준 좌표계 이름
        declare_parameter("camera_frame", "");                   // 비워두면 CameraInfo에서 자동으로 가져옴
        declare_parameter("mask_scale", 0.8);                    // 마스크 축소 비율(경계 오검출 방지)

        // 선언한 파라미터의 실제 값을 읽어 멤버 변수에 저장
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

        // YOLO 탐지기 생성 (엔진 로드는 여기서 한 번만 일어남)
        detector_ = std::make_shared<YoloDetector>(model_path, conf, iou);

        // 발행자(publisher) 3개 생성: 장애물 목록 / 디버그 이미지 / 물체 포인트클라우드
        pub_obstacles_ = create_publisher<hesai_zed_fusion::msg::ObstacleArray>(
            "hesai_zed/obstacles", 10);                  // 10 = 큐 크기(밀리면 오래된 것 버림)
        pub_overlay_ = create_publisher<sensor_msgs::msg::Image>(
            "hesai_zed/overlay", 10);
        pub_obj_pc_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            "hesai_zed/object_pointcloud", 10);

        // CameraInfo 구독: 콜백은 std::bind로 멤버 함수를 연결
        sub_camera_info_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            camera_info_topic, rclcpp::SensorDataQoS(),
            std::bind(&HesaiZedFusionNode::camera_info_callback, this, std::placeholders::_1));

        // 이미지 + 라이다는 message_filters로 구독 → 시간이 비슷한 짝끼리 묶어서 받음
        rmw_qos_profile_t sensor_qos = rmw_qos_profile_sensor_data;   // 센서용 QoS(빠르되 유실 허용)
        sensor_qos.depth = 10;
        sub_image_.subscribe(this, image_topic, sensor_qos);
        sub_lidar_.subscribe(this, lidar_topic, sensor_qos);

        // ApproximateTime 동기화: 타임스탬프가 "대략 같은" 이미지+라이다 쌍이 모이면
        // sync_callback을 호출한다 (두 센서는 정확히 같은 시각에 찍히지 않기 때문)
        sync_ = std::make_shared<Sync>(SyncPolicy(10), sub_image_, sub_lidar_);
        sync_->registerCallback(std::bind(&HesaiZedFusionNode::sync_callback, this,
            std::placeholders::_1, std::placeholders::_2));

        // COCO 클래스 번호 → 이름 매핑. 이 중 target_classes_만 장애물로 취급
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
    // CameraInfo 콜백: 최초 1회만 카메라 내부 파라미터 행렬 K(3x3)를 저장.
    // K는 "3D 점을 이미지 픽셀 좌표로 투영"할 때 쓰는 초점거리/중심점 정보.
    void camera_info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg) {
        if (!camera_info_received_) {
            K_ << msg->k[0], msg->k[1], msg->k[2],
                  msg->k[3], msg->k[4], msg->k[5],
                  msg->k[6], msg->k[7], msg->k[8];

            if (camera_frame_.empty()) {
                camera_frame_ = msg->header.frame_id;   // 파라미터로 안 정했으면 메시지에서 추출
            }
            camera_info_received_ = true;
        }
    }

    // ROS TF 메시지(이동+회전 쿼터니언) → 4x4 동차변환행렬로 변환하는 도우미.
    // 4x4 행렬 하나로 "회전 + 평행이동"을 한 번에 표현할 수 있다.
    Eigen::Matrix4f tf_to_eigen(const geometry_msgs::msg::TransformStamped& tf) {
        Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
        Eigen::Quaternionf q(
            tf.transform.rotation.w,
            tf.transform.rotation.x,
            tf.transform.rotation.y,
            tf.transform.rotation.z);
        T.block<3, 3>(0, 0) = q.toRotationMatrix();   // 좌상단 3x3 = 회전
        T(0, 3) = tf.transform.translation.x;         // 마지막 열 = 평행이동
        T(1, 3) = tf.transform.translation.y;
        T(2, 3) = tf.transform.translation.z;
        return T;
    }

    // 좌표 변환 행렬 캐싱: "라이다→카메라", "라이다→차량기준" 변환을 TF에서 한 번만
    // 조회해 저장한다 (센서 위치는 고정이므로 매 프레임 조회할 필요가 없음).
    bool cache_transforms() {
        if (transforms_cached_) return true;          // 이미 캐시됨 → 바로 성공
        if (camera_frame_.empty()) return false;      // 카메라 프레임 이름을 아직 모름

        try {
            auto tf_cam = tf_buffer_->lookupTransform(
                camera_frame_, lidar_frame_, tf2::TimePointZero);   // TimePointZero = 최신 값
            T_cam_lidar_ = tf_to_eigen(tf_cam);

            auto tf_base = tf_buffer_->lookupTransform(
                base_frame_, lidar_frame_, tf2::TimePointZero);
            T_base_lidar_ = tf_to_eigen(tf_base);

            transforms_cached_ = true;
            return true;
        } catch (tf2::TransformException& ex) {
            // TF가 아직 준비 안 됨 (부팅 직후 흔함) → 5초에 한 번만 경고 출력
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "TF not ready: %s", ex.what());
            return false;
        }
    }

    // ★ 핵심 콜백: 시간이 맞는 (이미지, 라이다) 쌍이 도착할 때마다 호출된다.
    //   흐름: 이미지 디코딩 → 클라우드 변환 → YOLO 탐지 → 라이다 점 투영
    //         → 마스크 안 점 매칭 → 최근접 장애물 좌표 계산 → 발행
    void sync_callback(
        const sensor_msgs::msg::CompressedImage::ConstSharedPtr& img_msg,
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr& lidar_msg)
    {
        // 준비물(K 행렬, TF)이 아직 없으면 이 프레임은 건너뜀
        if (!camera_info_received_ || !cache_transforms()) return;

        // JPEG 압축 바이트 → OpenCV 이미지로 디코딩
        cv::Mat raw_data(1, img_msg->data.size(), CV_8UC1, (void*)img_msg->data.data());
        cv::Mat cv_img = cv::imdecode(raw_data, cv::IMREAD_COLOR);
        if (cv_img.empty()) return;

        // ROS PointCloud2 메시지 → PCL 클라우드(점들의 배열)로 변환
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*lidar_msg, *cloud);
        if (cloud->empty()) return;

        // YOLO 추론 실행 → 탐지된 물체 목록
        auto results = detector_->detect(cv_img);

        // ---- 라이다 점 전체를 이미지 평면에 투영 ----
        std::vector<cv::Point> valid_uvs;      // 이미지 안에 찍힌 픽셀 좌표들
        std::vector<int> valid_indices;        // 그 점들의 원래 클라우드 인덱스
        std::vector<bool> is_object_point(cloud->points.size(), false);  // 물체 소속 여부 표시용

        for (size_t k = 0; k < cloud->points.size(); k++) {
            const auto& pt = cloud->points[k];
            Eigen::Vector4f p_lidar(pt.x, pt.y, pt.z, 1.0f);   // 동차좌표 (마지막 1은 변환용)
            Eigen::Vector4f p_cam = T_cam_lidar_ * p_lidar;    // 라이다 좌표 → 카메라 좌표

            if (p_cam(2) <= 0) continue;       // z<=0 = 카메라 뒤쪽 점 → 화면에 안 보임

            Eigen::Vector3f proj = K_ * p_cam.head<3>();       // 카메라 행렬 K로 투영
            if (proj(2) == 0) continue;

            // 동차좌표 나눗셈: (u,v) = 픽셀 좌표
            int u = static_cast<int>(proj(0) / proj(2));
            int v = static_cast<int>(proj(1) / proj(2));

            // 이미지 범위 안에 들어온 점만 후보로 저장
            if (u >= 0 && u < cv_img.cols && v >= 0 && v < cv_img.rows) {
                valid_uvs.emplace_back(u, v);
                valid_indices.push_back(k);
            }
        }

        // 발행할 장애물 배열 준비 (좌표계는 base_link = 차량 기준)
        hesai_zed_fusion::msg::ObstacleArray obs_array;
        obs_array.header = lidar_msg->header;
        obs_array.header.frame_id = base_frame_;

        // ---- 탐지된 물체마다: 마스크 안에 들어온 라이다 점을 찾아 거리 계산 ----
        for (auto& det : results) {
            if (det.box.area() <= 0) continue;

            std::string label = class_names_.count(det.class_id)
                ? class_names_[det.class_id] : "class_" + std::to_string(det.class_id);

            // 디버그 시각화: 이미지에 초록 박스와 클래스 이름을 그림
            cv::rectangle(cv_img, det.box, cv::Scalar(0, 255, 0), 2);
            cv::putText(cv_img, label, det.box.tl(), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);

            // 디버그 시각화: 마스크 영역을 빨간색 반투명으로 덧칠
            if (!det.mask.empty()) {
                cv::Rect valid_box = det.box & cv::Rect(0, 0, cv_img.cols, cv_img.rows);
                if (valid_box.area() > 0) {
                    cv::Mat colored_mask;
                    cv::cvtColor(det.mask, colored_mask, cv::COLOR_GRAY2BGR);
                    colored_mask.setTo(cv::Scalar(0, 0, 255), det.mask > 0);
                    cv::Mat roi = cv_img(valid_box);
                    cv::Mat mask_roi = colored_mask(cv::Rect(0, 0, valid_box.width, valid_box.height));
                    cv::addWeighted(roi, 1.0, mask_roi, 0.4, 0.0, roi);   // 40% 투명도로 섞기
                }
            }

            // 장애물 대상 클래스(사람/차/트럭...)가 아니거나 마스크가 없으면 거리 계산 생략
            bool is_target = false;
            for (int t : target_classes_) {
                if (det.class_id == t) { is_target = true; break; }
            }
            if (!is_target || det.mask.empty()) continue;

            // 마스크를 중심 기준으로 mask_scale(기본 0.8)배 축소 → 물체 경계에 걸친
            // 배경 라이다 점(바닥, 뒷벽 등)이 잘못 포함되는 것을 줄인다
            cv::Mat check_mask;
            int new_w = static_cast<int>(det.mask.cols * mask_scale_);
            int new_h = static_cast<int>(det.mask.rows * mask_scale_);

            if (new_w > 0 && new_h > 0) {
                check_mask = cv::Mat::zeros(det.mask.size(), det.mask.type());
                cv::Mat resized;
                cv::resize(det.mask, resized, cv::Size(new_w, new_h), 0, 0, cv::INTER_NEAREST);
                int ox = (det.mask.cols - new_w) / 2;   // 축소한 마스크를 가운데 배치
                int oy = (det.mask.rows - new_h) / 2;
                resized.copyTo(check_mask(cv::Rect(ox, oy, new_w, new_h)));
            } else {
                check_mask = det.mask;
            }

            // 마스크 안 라이다 점들 중 가장 가까운 점 찾기
            float min_dist = 9999.0f;
            Eigen::Vector3f closest_point(0, 0, 0);
            int point_count = 0;

            for (size_t k = 0; k < valid_uvs.size(); ++k) {
                const cv::Point& pt = valid_uvs[k];
                if (!det.box.contains(pt)) continue;    // 박스 밖 점은 볼 필요 없음

                // 픽셀 좌표를 박스 내부 상대 좌표(마스크 좌표)로 변환
                int mx = pt.x - det.box.x;
                int my = pt.y - det.box.y;

                if (mx >= 0 && mx < check_mask.cols && my >= 0 && my < check_mask.rows) {
                    if (check_mask.at<uint8_t>(my, mx) > 0) {   // 마스크가 켜진 픽셀 = 물체 위의 점
                        int idx = valid_indices[k];
                        is_object_point[idx] = true;
                        const auto& p3d = cloud->points[idx];
                        Eigen::Vector3f p(p3d.x, p3d.y, p3d.z);
                        float dist = p.norm();          // 라이다 원점부터의 직선거리
                        if (dist < min_dist) {
                            min_dist = dist;
                            closest_point = p;
                        }
                        point_count++;
                    }
                }
            }

            // 마스크 안에 라이다 점이 하나라도 있으면 장애물로 확정
            if (point_count > 0) {
                // 최근접점을 라이다 좌표 → 차량 기준(base_link) 좌표로 변환
                Eigen::Vector4f p_lidar(closest_point(0), closest_point(1), closest_point(2), 1.0f);
                Eigen::Vector4f p_base = T_base_lidar_ * p_lidar;

                hesai_zed_fusion::msg::ObstacleInfo obs;
                obs.label = label;
                obs.class_id = det.class_id;
                obs.confidence = det.conf;
                obs.rel_x = p_base(0);      // 차량 기준 상대 좌표 (x=앞, y=왼쪽, z=위)
                obs.rel_y = p_base(1);
                obs.rel_z = p_base(2);
                obs.distance = min_dist;
                obs_array.obstacles.push_back(obs);

                // 디버그 시각화: 박스 위에 "car 12.3m" 형태로 거리 표시
                std::stringstream ss;
                ss << std::fixed << std::setprecision(1) << min_dist;
                cv::putText(cv_img, label + " " + ss.str() + "m", cv::Point(det.box.x, det.box.y - 10), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
            }
        }

        // ---- 결과 발행 ----
        pub_obstacles_->publish(obs_array);                   // 장애물 목록 (핵심 출력)
        auto overlay_msg = cv_bridge::CvImage(img_msg->header, "bgr8", cv_img).toImageMsg();
        pub_overlay_->publish(*overlay_msg);                  // 박스/마스크 그려진 디버그 이미지

        // 물체에 속한 라이다 점만 모아서 별도 클라우드로 발행 (RViz 확인용)
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

    // 타입 별칭: "압축이미지 + 포인트클라우드를 근사 시각으로 묶는 동기화기"
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::CompressedImage, sensor_msgs::msg::PointCloud2>;
    using Sync = message_filters::Synchronizer<SyncPolicy>;

    // ---- 멤버 변수 (shared_ptr = 참조 카운트 방식 자동 메모리 관리 포인터) ----
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;          // TF 변환 저장소
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;  // TF 자동 수신기
    std::shared_ptr<YoloDetector> detector_;              // YOLO 탐지기
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_camera_info_;
    message_filters::Subscriber<sensor_msgs::msg::CompressedImage> sub_image_;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub_lidar_;
    std::shared_ptr<Sync> sync_;                          // 이미지+라이다 동기화기
    rclcpp::Publisher<hesai_zed_fusion::msg::ObstacleArray>::SharedPtr pub_obstacles_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_overlay_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_obj_pc_;

    Eigen::Matrix3f K_;                    // 카메라 내부 파라미터 행렬 (3D→픽셀 투영용)
    bool camera_info_received_ = false;    // K를 받았는가
    Eigen::Matrix4f T_cam_lidar_;          // 라이다 좌표 → 카메라 좌표 변환 행렬
    Eigen::Matrix4f T_base_lidar_;         // 라이다 좌표 → 차량 기준 좌표 변환 행렬
    bool transforms_cached_ = false;       // 변환 행렬을 캐시했는가
    std::string lidar_frame_;
    std::string base_frame_;
    std::string camera_frame_;
    float mask_scale_;                     // 마스크 축소 비율
    std::map<int, std::string> class_names_;   // 클래스 번호 → 이름
    std::vector<int> target_classes_;          // 장애물로 취급할 클래스 목록
};

// 프로그램 시작점: ROS2 초기화 → 노드 생성 → spin(콜백을 계속 처리하는 무한루프)
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HesaiZedFusionNode>());
    rclcpp::shutdown();
    return 0;
}
