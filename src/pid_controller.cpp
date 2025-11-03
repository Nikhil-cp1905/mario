#include "../include/pid_controller.hpp"

namespace control {

PIDController::PIDController(float kp, float ki, float kd,
                             float max_output, float min_output)
    : kp_(kp), ki_(ki), kd_(kd),
      max_output_(max_output), min_output_(min_output),
      prev_error_(0.0f), integral_(0.0f),
      prev_time_(std::chrono::steady_clock::now())
{
}

float PIDController::compute(float target, float current)
{
    auto now = std::chrono::steady_clock::now();
    float dt = std::chrono::duration<float>(now - prev_time_).count();
    prev_time_ = now;  //Measure elapsed time (dt)

    if (dt <= 0.0f) {
        dt = 1e-3f; //This ensures dt is never zero, so we don’t divide by zero later.

    }

    float error = target - current;  //Calculate error terms
    integral_ += error * dt;
    float derivative = (error - prev_error_) / dt;
    prev_error_ = error;

    float output = kp_ * error + ki_ * integral_ + kd_ * derivative;  //each k is gain
    return std::clamp(output, min_output_, max_output_);
}

void PIDController::reset()
{
    prev_error_ = 0.0f; //
    integral_ = 0.0f; 
}

DiffDriveTwist compute_pid_from_slam(
    const Slam_Pose& current_pose,
    const Slam_Pose& target_pose,
    PIDController& linear_pid,
    PIDController& angular_pid
)
{
    DiffDriveTwist cmd{0.0f, 0.0f}; //Initialization

    float dx = target_pose.x - current_pose.x; //Calculate distance and heading difference
    float dy = target_pose.y - current_pose.y;
    float distance_error = std::sqrt(dx * dx + dy * dy);

    float target_yaw = std::atan2(dy, dx); //Compute target heading (yaw)
    float yaw_error = target_yaw - current_pose.yaw;

    while (yaw_error > M_PI) {
        yaw_error -= 2.0f * M_PI;
    }
    while (yaw_error < -M_PI) {
        yaw_error += 2.0f * M_PI;  //Normalize yaw error (−π to π)
    }

    cmd.linear_x = linear_pid.compute(0.0f, -distance_error); //Use PID controllers
    cmd.angular_z = angular_pid.compute(0.0f, -yaw_error);

    return cmd; //Return final motion command
}

}
