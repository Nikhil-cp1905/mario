#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include <chrono>
#include <cmath>
#include <algorithm>

namespace control {

struct Slam_Pose {
    float x;
    float y;
    float yaw;
};

struct PIDController {
    float kp, ki, kd;
    float max_output, min_output;
    float prev_error, integral;
    std::chrono::steady_clock::time_point prev_time;

    PIDController(float kp, float ki, float kd,
                  float max_output = 1.0f,
                  float min_output = -1.0f);

    float compute(float target, float current);
    void reset();
};

struct DiffDriveTwist {
    float linear_x;
    float angular_z;
};

DiffDriveTwist compute_pid_from_slam(
    const Slam_Pose& current_pose,
    const Slam_Pose& target_pose,
    PIDController& linear_pid,
    PIDController& angular_pid
);

} // namespace control

#endif
