#include "cdr_monitor/calculate_ftti.hpp"
#include <tf2/utils.h>
#include <iostream>
#include <string.h>
using namespace std;

double CalculateFtti::calculate_vel(double vel_x, double vel_y) {
    return sqrt((vel_x * vel_x) + (vel_y * vel_y));
}

double CalculateFtti::calculate_ttc(double vel, double dist){
    if (vel != 0)
        return (dist/vel) * 1000;
    else
        return 1000.0;
}
vector<string> CalculateFtti::parsing_port(string raw){
    vector<string> result = {};
    size_t start = 0;
    size_t end = raw.find(',');
    while (end != string::npos){
        result.push_back(raw.substr(start,end-start));
        start = end+1;
        end = raw.find(',', start);
    }
    result.push_back(raw.substr(start));
    return result;
}

CalculateFtti::CalculateFtti(const string& name, const NodeConfig& config)
    : SyncActionNode(name,config)
{}

PortsList CalculateFtti::providedPorts() {
    return {
        InputPort<vector<string>>("lidar_names"),
        InputPort<vector<string>>("camera_names"),
        InputPort<double>("vel_x"),
        InputPort<double>("vel_y"),
        InputPort<double>("nearest_obstacle"),
        InputPort<double>("ttc_ext_ms"),
        InputPort<string>("important_sensors"),
        InputPort<string>("ttc_source_sensors"),
        // 데드라인 상한(기존 하드코딩 850ms)과 TTC 동적 조임의 파라미터화
        InputPort<double>("fresh_ms", 850.0, "sensor freshness deadline upper bound [ms]"),
        InputPort<bool>("ttc_dynamic", true, "min(fresh, TTC) dynamic tightening on/off"),
    };
}
NodeStatus CalculateFtti::tick() {
    double vel_x = 0.0;
    double vel_y = 0.0;
    double nearest_obstacle = 1000.0;
    string important_sensors = "";
    vector<string> lidar_names;
    vector<string> camera_names;

    getInput("vel_x", vel_x);
    getInput("vel_y", vel_y);
    double vel = calculate_vel(vel_x, vel_y);

    // fresh_ms: 데드라인 상한 (기본 850ms). ttc_dynamic=false면 TTC로 조이지 않음
    double fresh_ms = 850.0;
    bool ttc_dynamic = true;
    getInput("fresh_ms", fresh_ms);
    getInput("ttc_dynamic", ttc_dynamic);

    getInput("important_sensors", important_sensors);

    getInput("nearest_obstacle", nearest_obstacle);

    if (nearest_obstacle <= 0.0){
        nearest_obstacle = 100.0;
    }

    double ttc = calculate_ttc(vel, nearest_obstacle);

    // spatio-temporal TTC from planned trajectory x predicted object paths:
    // catches crossing/oncoming encounters the straight-line estimate misses
    double ttc_ext = -1.0;
    getInput("ttc_ext_ms", ttc_ext);
    if (ttc_ext > 0.0 && ttc_ext < ttc)
        ttc = ttc_ext;

    vector<string> important_sensors_names;
    if (!important_sensors.empty())
        important_sensors_names = parsing_port(important_sensors);

    cout << "dist " << nearest_obstacle << "m" << endl;
    cout << "ttc " << ttc << "ms (ext " << ttc_ext << "ms)" << endl;

    map<string,double> ftti_map;

    getInput("lidar_names", lidar_names);
    getInput("camera_names", camera_names);

    auto bb = config().blackboard;

    // Fail-conservative TTC: the obstacle list comes from the perception
    // pipeline, so when its source sensors are unhealthy a *growing* TTC may
    // just mean missed detections. Hold the last trusted value and only take
    // the current one if it is smaller (more conservative). Sources default
    // to all lidars (AWCL objects are lidar-pipeline based).
    string ttc_sources = "";
    getInput("ttc_source_sensors", ttc_sources);
    vector<string> source_names =
        ttc_sources.empty() ? lidar_names : parsing_port(ttc_sources);
    bool source_healthy = true;
    for (const auto& n : source_names) {
        bool failed = false;
        double q = 1.0;
        (void)bb->get(n + "_failed", failed);
        (void)bb->get(n + "_quality_score", q);
        if (failed || q < 0.7) { source_healthy = false; break; }
    }
    // TTC 하한 감쇠 (Emzivat 방향): 마지막 신뢰 TTC를 얼려두지 않고 경과시간만큼
    // 감소시킨다 — ttc_hold(t) = ttc_last − elapsed_ms.
    // worst-case propagation: 마지막 관측 장애물이 같은 접근속도로 계속 다가온다고
    // 가정. 0 이하로 떨어지면 최대 감점(TTC=0 → FTTI=0)을 한 번 적용한 뒤 홀드를
    // 해제한다 — 지속 고장은 품질 점수가 이미 벌점하므로 영구 홀드는 이중 처벌.
    if (source_healthy) {
        trusted_ttc_ = ttc;
        was_unhealthy_ = false;
        hold_expired_ = false;
    } else {
        if (!was_unhealthy_) {
            was_unhealthy_ = true;
            hold_expired_ = false;
            unhealthy_since_ = std::chrono::steady_clock::now();
        }
        const double elapsed_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - unhealthy_since_).count();
        const double ttc_hold = trusted_ttc_ - elapsed_ms;
        if (!hold_expired_) {
            if (ttc_hold <= 0.0) {
                cout << "ttc hold decayed to 0: max penalty, then release" << endl;
                ttc = 0.0;              // 최대 감점 후
                hold_expired_ = true;   // 홀드 해제 (이후 측정값 사용)
            } else if (ttc_hold < ttc) {
                cout << "ttc source unhealthy: hold " << ttc_hold
                     << "ms (measured " << ttc << "ms, decayed " << elapsed_ms
                     << "ms)" << endl;
                ttc = ttc_hold;
            }
        }
    }

    for (const auto& name : lidar_names) {
        ftti_map[name] = fresh_ms;
    }
    for (const auto& name : camera_names) {
        ftti_map[name] = fresh_ms;
    }
    // ttc_dynamic=true일 때만 중요 센서의 데드라인을 min(fresh_ms, TTC)로 조임
    if (ttc_dynamic) {
        for (const auto& name : important_sensors_names) {
            if (ttc < fresh_ms)
                ftti_map[name] = ttc;
        }
    }

    for (auto& [name,value] : ftti_map){
        bb->set(name + "_ftti", value);
        RCLCPP_INFO(rclcpp::get_logger("ftti_names"), "%s : %.2f", name.c_str(), value);
    }

    return NodeStatus::SUCCESS;
}
