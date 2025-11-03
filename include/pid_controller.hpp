#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <chrono>
#include <cmath>
#include <algorithm>

struct SlamPose {
    float x;
    float y;
    float yaw;
};

class PIDController {
public:
    PIDController(float kp, float ki, float kd,
                  float max_output = 1.0f,
                  float min_output = -1.0f);

    float compute(float target, float current);
    void reset();

private:
    float kp_, ki_, kd_;
    float max_output_, min_output_;
    float prev_error_, integral_;
    std::chrono::steady_clock::time_point prev_time_;
};

struct RoverControlCommand {
    float linear_velocity;
    float angular_velocity;
};

RoverControlCommand compute_pid_from_slam(
    const SlamPose& current_pose,
    const SlamPose& target_pose,
    PIDController& linear_pid,
    PIDController& angular_pid
);

}

#endif

