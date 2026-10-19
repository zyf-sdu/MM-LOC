#pragma once
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <iostream>

// Optional instrumentation. The runner creates a unique directory before launch.
namespace resource_metrics {
inline double steady_now() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
inline bool enabled() {
    const char* directory = std::getenv("BMLL_METRICS_DIR");
    return directory && *directory;
}
// Each process writes distinct module files. CLOCK_MONOTONIC/steady_clock on
// this Linux host is shared across processes; ROS bag/simulation time is not
// used for elapsed time. stamp is only the exact frame association key.
inline void event(const std::string& module, double stamp, const std::string& kind,
                  double value = 0, double when = -1) {
    if (!enabled()) return;
    if (when < 0) when = steady_now();
    static std::mutex mutex;
    static std::map<std::string, std::unique_ptr<std::ofstream>> files;
    std::lock_guard<std::mutex> lock(mutex);
    auto& file = files[module];
    if (!file) {
        file.reset(new std::ofstream(std::string(std::getenv("BMLL_METRICS_DIR")) + "/" + module + "_events.csv", std::ios::app));
        if (!*file) { std::cerr << "Cannot open event log: " << module << std::endl; return; }
        if (file->tellp() == 0) *file << "stamp,steady_sec,event,value\n";
        *file << std::fixed << std::setprecision(9);
    }
    *file << stamp << ',' << when << ',' << kind << ',' << value << '\n';
    // Nodes may leave via ROS signal handlers without destroying these streams.
    // Persist every event so the final poses and their receipt keys cannot be lost.
    file->flush();
}
inline void pose(double output_stamp, double input_stamp, double received, double published) {
    // Two events with the exact output stamp join the original LiDAR callback
    // entry to /Odometry publication and downstream fused-pose publication.
    event("fast_lio_pose", output_stamp, "receive", input_stamp, received);
    event("fast_lio_pose", output_stamp, "publish", (published-received)*1000.0, published);
}
inline void record(const std::string& module, double stamp, double ms) {
    const char* directory = std::getenv("BMLL_METRICS_DIR");
    if (!directory || !*directory) return;
    static std::mutex mutex;
    static std::map<std::string, std::unique_ptr<std::ofstream>> files;
    std::lock_guard<std::mutex> lock(mutex);
    auto& file = files[module];
    if (!file) {
        file.reset(new std::ofstream(std::string(directory) + "/" + module + ".csv", std::ios::app));
        if (!*file) { std::cerr << "Cannot open timing log: " << module << std::endl; return; }
        if (file->tellp() == 0) *file << "stamp,steady_sec,total_ms\n";
        *file << std::fixed << std::setprecision(9);
    }
    const double steady = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
    *file << stamp << ',' << steady << ',' << ms << '\n';
    file->flush();
}
class Scope {
    std::string module_;
    double stamp_;
    std::chrono::steady_clock::time_point start_;
public:
    Scope(std::string module, double stamp): module_(std::move(module)), stamp_(stamp), start_(std::chrono::steady_clock::now()) {}
    ~Scope() { record(module_, stamp_, std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now()-start_).count()); }
};
}
