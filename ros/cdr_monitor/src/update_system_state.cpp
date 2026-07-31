/**
 * [update_system_state.cpp] "중요 센서" 선정 노드.
 *
 * 전체 시스템에서의 역할:
 *   차의 현재 속도/방향으로 앞으로 1.5초간 지나갈 경로를 예측하고,
 *   그 경로가 각 센서의 시야(FOV, Field of View) 안에 들어오는지 검사한다.
 *   경로를 "보고 있는" 센서 = 지금 위험과 직결된 "중요 센서".
 *
 * 데이터 흐름:
 *   블랙보드에서 vel_x/vel_y/heading 읽기
 *   → 등속·등회전 모델로 미래 경로점 15개 생성 (RViz용 마커도 발행)
 *   → 센서마다 isPathInFov() 검사 → 중요 센서 목록을 쉼표 문자열로
 *   → important_sensors 포트(블랙보드)에 출력 → CalculateFtti가 이 센서들의
 *     FTTI만 TTC로 조여준다. 시각화용 FOV 부채꼴 마커도 발행.
 */
#include "cdr_monitor/update_system_state.hpp"
#include <tf2/utils.h>                                  // 쿼터니언 → yaw 각도 변환 등 기하 도우미
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>      // geometry_msgs 타입을 tf2에서 쓰게 하는 어댑터
#include <iostream>
using namespace std;

// 생성자: 이 BT 노드가 만들어질 때 한 번 실행. ROS 발행자/TF 준비 + 센서 설정 읽기.
UpdateSystemState::UpdateSystemState(const string& name, const NodeConfig& config,
                                        const BT::RosNodeParams& params)
    : SyncActionNode(name,config)   // 부모(동기식 액션 노드) 생성자에 이름/설정 전달
{
    // params.nh 는 weak_ptr(소유권 없는 약한 참조)라서 lock()으로
    // 실제 shared_ptr(자동 소유권 관리 포인터)를 꺼내 써야 한다.
    node_= params.nh.lock();
    // publisher = "방송 송출기". 이 토픽으로 메시지를 내보내면 구독자들이 받는다.
    // 예측 경로를 RViz에 선으로 그리기 위한 마커 발행자 (큐 크기 10)
    path_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/kinematic_path", 10);
    // 센서 시야(부채꼴)를 RViz에 그리기 위한 마커 발행자
    fov_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/sensor_fov", 10);

    // TF = ROS의 좌표계 변환 시스템. "센서가 차의 어디에 어떤 방향으로 붙어있나"를
    // 다른 노드들이 방송해 주는데, buffer가 그걸 모아두고 listener가 구독한다.
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // 시야(FOV)를 가진 센서 종류들: 라이다/카메라/레이더 (IMU/GPS는 시야 개념이 없음)
    vector<string> fov_types = {"lidar", "camera", "radar"};
    for (const auto& type : fov_types) {
        // 설정 파일에서 이 종류의 센서 이름 목록 읽기 (main.cpp에서 이미 declare됨)
        vector<string> names;
        node_->get_parameter("sensors." + type + ".names", names);

        for (const auto& sensor_name : names) {
            // 예: "sensors.lidar.Hesai_AT128." 접두사로 세부 설정을 읽는다
            string prefix = "sensors." + type + "." + sensor_name + ".";
            // 파라미터는 declare(선언) 후 get(읽기) — ROS2 규칙
            node_->declare_parameter<string>(prefix + "frame_id", "");            // 센서의 TF 좌표계 이름
            node_->declare_parameter<string>(prefix + "ns", "");                  // RViz 마커 네임스페이스
            node_->declare_parameter<double>(prefix + "h_fov_deg", 0.0);          // 수평 시야각 [도]
            node_->declare_parameter<double>(prefix + "range", 0.0);              // 감지 거리 [m]
            node_->declare_parameter<std::vector<double>>(prefix + "color", {0,0,0,0});   // 마커 색 RGBA

            // 센서 하나의 시야 스펙을 구조체에 채워 넣는다
            SensorFovSpec spec;
            spec.name = sensor_name;
            node_->get_parameter(prefix + "frame_id", spec.frame_id);
            node_->get_parameter(prefix + "ns", spec.ns);
            node_->get_parameter(prefix + "h_fov_deg", spec.h_fov_deg);
            node_->get_parameter(prefix + "range", spec.range);

            std::vector<double> color;
            node_->get_parameter(prefix + "color", color);
            spec.r = color[0]; spec.g = color[1];
            spec.b = color[2]; spec.a = color[3];

            sensors_.push_back(spec);   // 전체 센서 목록에 추가 (tick마다 순회할 대상)
        }
    }
}

// 이 노드의 포트 선언: 속도/방향은 입력, 중요 센서 목록은 출력.
// OutputPort = 계산 결과를 블랙보드에 써 넣는 출력 칸.
PortsList UpdateSystemState::providedPorts() {
    return {
        InputPort<double>("vel_x"),
        InputPort<double>("vel_y"),
        InputPort<double>("current_heading"),
        OutputPort<string>("important_sensors"),
    };
}

// "예측 경로의 점들 중 하나라도 이 센서의 시야(부채꼴) 안에 있나?" 검사.
// 있으면 true → 그 센서는 지금 "중요 센서"다.
bool UpdateSystemState::isPathInFov(
    const vector<geometry_msgs::msg::Point>& path_points,
    const SensorFovSpec& spec)
{
    // 센서가 차체(base_link) 기준 어디에/어느 방향으로 붙어 있는지 TF에서 조회
    geometry_msgs::msg::TransformStamped tf;
    try {
        tf = tf_buffer_->lookupTransform(
            "base_link", spec.frame_id, tf2::TimePointZero);   // TimePointZero = "가장 최근 값"
    } catch (const tf2::TransformException&) {
        return false;   // 변환 정보가 아직 없으면 판단 불가 → 중요 아님으로 처리
    }

    // 센서의 (차체 기준) 위치와 바라보는 방향(yaw)
    double sx = tf.transform.translation.x;
    double sy = tf.transform.translation.y;
    double sensor_yaw = tf2::getYaw(tf.transform.rotation);   // 쿼터니언 → 평면 회전각 [rad]

    // 시야각의 절반 (도 → 라디안 변환 포함). 부채꼴 중심선 기준 ±half_fov 안이면 시야 안.
    double half_fov = spec.h_fov_deg * M_PI / 180.0 / 2.0;

    for (const auto& pt : path_points) {
        // 센서 → 경로점 벡터와 거리
        double dx = pt.x - sx;
        double dy = pt.y - sy;
        double dist = sqrt(dx * dx + dy * dy);

        if (dist > spec.range) continue;   // 감지 거리 밖이면 이 점은 패스

        // 경로점이 센서 정면에서 몇 라디안 벗어나 있는지
        double angle = atan2(dy, dx) - sensor_yaw;
        // 각도를 -180°~+180° 범위로 정규화 (한 바퀴 넘어가는 값 정리)
        while (angle > M_PI)  angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;

        // 시야각 안에 들어오면: 이 센서는 예측 경로를 보고 있다!
        if (fabs(angle) <= half_fov) {
            return true;
        }
    }
    return false;   // 어떤 경로점도 시야에 안 들어옴
}

// RViz에 그릴 센서 시야 부채꼴 마커 생성.
// 상태에 따라 색이 달라짐: 고장=굵은 빨강, 중요=주황, 평상시=설정 색.
visualization_msgs::msg::Marker UpdateSystemState::createFovMarker(
    const SensorFovSpec& spec, int id, const rclcpp::Time& stamp,
    bool is_important, bool is_failed)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = spec.frame_id;   // 이 좌표계(센서 위치) 기준으로 그려라
    marker.header.stamp = stamp;
    marker.ns = spec.ns;                      // RViz에서 그룹으로 묶이는 이름
    marker.id = id;                           // 같은 ns 안에서 마커 구분 번호
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;   // 점들을 잇는 선
    marker.action = visualization_msgs::msg::Marker::ADD;        // 추가/갱신

    if (is_failed) {                 // 고장난 센서: 굵고 새빨간 선 (눈에 확 띄게)
        marker.scale.x = 0.05;       // 선 굵기
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;       // 불투명
    } else if (is_important) {       // 중요 센서: 주황색
        marker.scale.x = 0.03;
        marker.color.r = 1.0f;
        marker.color.g = 0.5f;
        marker.color.b = 0.0f;
        marker.color.a = 0.9f;
    } else {                         // 평상시: 설정 파일에 적힌 고유 색, 얇은 선
        marker.scale.x = 0.01;
        marker.color.r = spec.r;
        marker.color.g = spec.g;
        marker.color.b = spec.b;
        marker.color.a = spec.a;
    }

    // 부채꼴 호(arc)를 40개 직선 조각으로 근사해서 그린다
    double h_fov_rad = spec.h_fov_deg * M_PI / 180.0;
    double start_angle = -h_fov_rad / 2.0;   // 부채꼴 왼쪽 끝 각도
    double end_angle   =  h_fov_rad / 2.0;   // 오른쪽 끝 각도
    int segments = 40;
    double angle_step = (end_angle - start_angle) / segments;

    // 부채꼴 꼭짓점(센서 위치 = 원점)에서 시작
    geometry_msgs::msg::Point origin;
    origin.x = 0.0; origin.y = 0.0; origin.z = 0.0;
    marker.points.push_back(origin);

    // 호를 따라 점들을 찍는다 (반지름 = 감지 거리)
    for (int i = 0; i <= segments; i++) {
        double a = start_angle + i * angle_step;
        geometry_msgs::msg::Point p;
        p.x = spec.range * cos(a);
        p.y = spec.range * sin(a);
        p.z = 0.0;
        marker.points.push_back(p);
    }

    // 마지막에 다시 원점으로 돌아와 부채꼴을 닫는다
    marker.points.push_back(origin);

    return marker;
}

// 매 틱(10Hz)마다 호출되는 본체.
NodeStatus UpdateSystemState::tick() {
    // 블랙보드에서 현재 속도/방향 읽기
    double vel_x = 0.0, vel_y = 0.0;
    double current_heading = 0.0;
    getInput("vel_x", vel_x);
    getInput("vel_y", vel_y);
    getInput("current_heading", current_heading);

    double current_vel = std::hypot(vel_x, vel_y);   // hypot = sqrt(x²+y²), 속력

    // ── 회전 속도(yaw rate) 추정: heading이 시간당 얼마나 변했나 ──
    rclcpp::Time now = node_->now();
    double yaw_rate = 0.0;

    if (first_tick_) {
        // 첫 틱에는 "이전 값"이 없어서 변화율을 못 구함 → 기록만 하고 넘어감
        first_tick_ = false;
    } else {
        double dt_real = (now - prev_time_).seconds();   // 직전 틱과의 실제 시간 간격
        if (dt_real > 0.001) {                            // 0에 가까운 dt로 나누기 방지
            double dh = current_heading - prev_heading_;  // 방향 변화량
            // ±180° 경계를 넘는 순간(예: 179° → -179°)을 정상 변화로 정규화
            while (dh > M_PI)  dh -= 2.0 * M_PI;
            while (dh < -M_PI) dh += 2.0 * M_PI;
            yaw_rate = -dh / dt_real;                     // 변화량/시간 = 회전 속도 [rad/s]
        }
    }
    // 다음 틱을 위해 현재 값을 "이전 값"으로 저장
    prev_heading_ = current_heading;
    prev_time_ = now;

    // ── 미래 경로 예측: "지금 속력과 회전 속도가 유지된다"고 가정하고 굴려본다 ──
    // 차체 기준 좌표계라 시작점은 항상 (0,0), 시작 방향은 0.
    double x = 0.0, y = 0.0;
    double heading = 0.0;
    double dt = 0.1;    // 한 스텝 = 0.1초
    int steps = 15;     // 15스텝 → 1.5초 앞까지 내다본다

    vector<geometry_msgs::msg::Point> path_points;          // FOV 검사에 쓸 경로점들
    visualization_msgs::msg::MarkerArray path_array;        // RViz 발행용 묶음
    visualization_msgs::msg::Marker line_strip;             // 경로를 그릴 선 마커
    line_strip.header.frame_id = "base_link";               // 차체 좌표계 기준
    line_strip.header.stamp = now;
    line_strip.ns = "prediction";
    line_strip.id = 0;
    line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line_strip.action = visualization_msgs::msg::Marker::ADD;

    line_strip.scale.x = 0.15;   // 선 굵기
    line_strip.color.r = 0.0; line_strip.color.g = 0.0; line_strip.color.b = 1.0;   // 파란색
    line_strip.color.a = 0.8;

    // 시작점 (차 위치, 살짝 띄워서 z=0.1 — 바닥에 파묻히지 않게)
    geometry_msgs::msg::Point start;
    start.x = 0.0; start.y = 0.0; start.z = 0.1;
    line_strip.points.push_back(start);
    path_points.push_back(start);

    // 등속·등회전 운동 모델로 한 스텝씩 전진시키며 경로점 생성
    for (int i = 0; i < steps; i++) {
        heading += yaw_rate * dt;                  // 방향이 조금씩 돌아가고
        x += current_vel * cos(heading) * dt;      // 그 방향으로 속력만큼 이동
        y += current_vel * sin(heading) * dt;

        geometry_msgs::msg::Point p;
        p.x = x;
        p.y = y;
        p.z = 0.1;
        line_strip.points.push_back(p);
        path_points.push_back(p);
    }

    // 예측 경로를 RViz로 발행 (publish = 방송 내보내기)
    path_array.markers.push_back(line_strip);
    path_pub_->publish(path_array);

    // ── 센서별로 "경로를 보고 있나?" 검사 → 중요 센서 목록 작성 + FOV 마커 생성 ──
    visualization_msgs::msg::MarkerArray fov_array;
    vector<string> important_list;

    auto bb = config().blackboard;   // 블랙보드 직접 접근 핸들
    for (size_t i = 0; i < sensors_.size(); i++) {
        bool important = isPathInFov(path_points, sensors_[i]);   // 시야 안에 경로가 있나?
        if (important) {
            important_list.push_back(sensors_[i].name);
        }
        // 검사 노드들이 블랙보드에 적어둔 고장 여부도 읽는다 (마커 색 결정용)
        bool failed = false;
        bb->get(sensors_[i].name + "_failed", failed);
        fov_array.markers.push_back(
            createFovMarker(sensors_[i], static_cast<int>(i), now, important, failed));
    }
    fov_pub_->publish(fov_array);   // 시야 부채꼴들을 RViz로 발행

    // 중요 센서 목록을 "A,B,C" 형태의 쉼표 문자열로 합친다
    // (블랙보드 포트로 넘기기 쉽게 문자열 하나로 만드는 것)
    string important_str;
    for (size_t i = 0; i < important_list.size(); i++) {
        if (i > 0) important_str += ",";
        important_str += important_list[i];
    }

    // 출력 포트에 기록 → 블랙보드의 important_sensors 갱신 → CalculateFtti가 사용
    setOutput("important_sensors", important_str);
    cout << "-------------------------------" << endl;
    cout << "[CDR Monitor] Important Sensors: "
         << (important_str.empty() ? "None" : important_str) << endl;

    return NodeStatus::SUCCESS;   // 성공 반환 → 트리가 다음 노드(CalculateFtti)로 진행
}
