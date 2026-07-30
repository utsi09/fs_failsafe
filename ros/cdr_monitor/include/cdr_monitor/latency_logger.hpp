#pragma once
#include <fstream>
#include <mutex>
#include <chrono>
#include <string>
#include <cstdint>

class LatencyLogger {
public:
    static LatencyLogger& instance() {
        static LatencyLogger inst;
        return inst;
    }

    void init(const std::string& path) {
        std::lock_guard<std::mutex> lk(mu_);
        if (!f_.is_open()) {
            f_.open(path);
            f_ << "sensor,stamp_ns,wall_recv_ns,wall_tick_ns,staleness_us\n";
        }
    }

    void log(const std::string& sensor, int64_t stamp_ns,
             int64_t wall_recv_ns) {
        int64_t wall_tick_ns =
            std::chrono::steady_clock::now().time_since_epoch().count();
        double staleness_us = (wall_tick_ns - wall_recv_ns) / 1000.0;

        std::lock_guard<std::mutex> lk(mu_);
        f_ << sensor << "," << stamp_ns << ","
           << wall_recv_ns << "," << wall_tick_ns << ","
           << staleness_us << "\n";
        if (++count_ % 100 == 0) f_.flush();
    }

    ~LatencyLogger() { if (f_.is_open()) { f_.flush(); f_.close(); } }

private:
    LatencyLogger() = default;
    std::ofstream f_;
    std::mutex mu_;
    int count_ = 0;
};
