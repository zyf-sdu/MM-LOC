#pragma once
#include <Eigen/Geometry>
#include <mutex>

// Only completed, accepted backend corrections are visible to publishers.
// Never hold this mutex over optimization, history updates, ROS or file I/O.
class GlobalCorrectionSnapshot {
    mutable std::mutex mutex_;
    Eigen::Matrix4d correction_ = Eigen::Matrix4d::Identity();
public:
    void update(const Eigen::Matrix4d& correction) {
        std::lock_guard<std::mutex> lock(mutex_);
        correction_ = correction;
    }
    Eigen::Matrix4d read() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return correction_;
    }
    void project(const Eigen::Matrix4d& odometry, Eigen::Vector3d& position,
                 Eigen::Quaterniond& orientation) const {
        const Eigen::Matrix4d correction = read();
        // Same composition as inputOdom(), for this frame rather than lastP/Q.
        position = correction.block<3,3>(0,0) * odometry.block<3,1>(0,3)
                 + correction.block<3,1>(0,3);
        orientation = correction.block<3,3>(0,0)
                    * Eigen::Quaterniond(odometry.block<3,3>(0,0));
    }
};
