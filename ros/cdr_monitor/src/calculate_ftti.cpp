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
    // Hold is time-limited: it bridges transient detection dropouts, but a
    // persistent fault is already punished by the quality score — keeping a
    // frozen small TTC forever double-penalizes and collapses the score
    // below the DEFER floor (band design breaks: shoulder pull-over never
    // gets to run).
    if (source_healthy) {
        trusted_ttc_ = ttc;
        was_unhealthy_ = false;
    } else {
        if (!was_unhealthy_) {
            was_unhealthy_ = true;
            unhealthy_since_ = std::chrono::steady_clock::now();
        }
        const double held_s = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - unhealthy_since_).count();
        if (held_s < 5.0 && trusted_ttc_ < ttc) {
            cout << "ttc source unhealthy: hold " << trusted_ttc_
                 << "ms (measured " << ttc << "ms, " << held_s << "s)" << endl;
            ttc = trusted_ttc_;
        }
    }

    for (const auto& name : lidar_names) {
        ftti_map[name] = 850.0;
    }
    for (const auto& name : camera_names) {
        ftti_map[name] = 850.0;
    }
    for (const auto& name : important_sensors_names) {
        if (ttc < 850.0)
            ftti_map[name] = ttc;
    }

    for (auto& [name,value] : ftti_map){
        bb->set(name + "_ftti", value);
        RCLCPP_INFO(rclcpp::get_logger("ftti_names"), "%s : %.2f", name.c_str(), value);
    }

    return NodeStatus::SUCCESS;
}
