#pragma once
#include <Eigen/Geometry>
#include <mutex>

// This mutex only protects a small value copy, never matching or optimization.
class OdometryCorrectionSnapshot {
    mutable std::mutex mutex_;
    Eigen::Affine3f correction_ = Eigen::Affine3f::Identity();
    bool ready_ = false;
public:
    void update(const Eigen::Affine3f& correction, bool ready) {
        std::lock_guard<std::mutex> lock(mutex_);
        correction_ = correction;
        ready_ = ready;
    }
    bool read(Eigen::Affine3f& correction) const {
        std::lock_guard<std::mutex> lock(mutex_);
        correction = correction_;
        return ready_;
    }
};

// Owned only by the matching worker. Use odometry motion, not map corrections,
// so a correction jump cannot manufacture a new keyframe while stationary.
class MatchingKeyframeGate {
    Eigen::Vector3f last_ = Eigen::Vector3f::Zero();
    bool have_last_ = false;
public:
    bool shouldMatch(const Eigen::Vector3f& position, float distance, bool force=false) const {
        return force || !have_last_ || (position-last_).norm() > distance;
    }
    void commit(const Eigen::Vector3f& position) { last_=position; have_last_=true; }
};
