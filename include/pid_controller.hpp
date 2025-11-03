#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include <chrono>
#include <cmath>
#include <algorithm>

//struct for SlamPose, x and y for pos , annd yaw for orientation,same pressing in pavithra's mapping.h in her slam repo
struct Slam_Pose {
    float x;
    float y;
    float yaw;
};



class PIDController {
public:
    PIDController(float kp, float ki, float kd, // kp -propotional gain to react to current error ,ki - integral gain to eliminated steady state error , kd- derivative gain , dampens oscillation
                  float max_output = 1.0f,
                  float min_output = -1.0f); //clamps output  can be changed later

    float compute(float target, float current); //calculate pid
    void reset();

private:
    float kp_, ki_, kd_; //provided aboe
    float max_output_, min_output_;
    float prev_error_, integral_; //prev error , last error for derivative calculation , intergral accumulates error over time    
    std::chrono::steady_clock::time_point prev_time_;  //prev time  calculates delta
};

struct DiffDriveTwist {
    float linear_x; //forw,back
    float angular_z; //rotation
};

RoverControlCommand compute_pid_from_slam(
    const SlamPose& current_pose,
    const SlamPose& target_pose,
    PIDController& linear_pid,
    PIDController& angular_pid
);
// SLAM gives you current position → Compare with target → PID calculates corrections → Send velocities to motors → Repeat
}

#endif

