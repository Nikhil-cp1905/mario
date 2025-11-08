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
#ifdef PID_TEST_CPP
#include <iostream>
#include <iomanip>
#include <vector>

using namespace control;

// Simple simulation for testing PID parameters
struct SimulationState {
    mapping::Slam_Pose current;
    mapping::Slam_Pose target;
    float velocity_x = 0.0f;
    float velocity_z = 0.0f;
};

void simulate_step(SimulationState& state, const DiffDriveTwist& cmd, float dt) {
    // Simple kinematic model
    state.velocity_x = cmd.linear_x;
    state.velocity_z = cmd.angular_z;
    
    // Update position
    state.current.x += state.velocity_x * cos(state.current.yaw) * dt;
    state.current.y += state.velocity_x * sin(state.current.yaw) * dt;
    state.current.yaw += state.velocity_z * dt;
    
    // Normalize yaw
    while (state.current.yaw > M_PI) state.current.yaw -= 2.0f * M_PI;
    while (state.current.yaw < -M_PI) state.current.yaw += 2.0f * M_PI;
}

void test_pid_parameters(float kp_lin, float ki_lin, float kd_lin,
                        float kp_ang, float ki_ang, float kd_ang) {
    
    std::cout << "\n========================================\n";
    std::cout << "Testing PID Parameters:\n";
    std::cout << "Linear:  Kp=" << kp_lin << " Ki=" << ki_lin << " Kd=" << kd_lin << "\n";
    std::cout << "Angular: Kp=" << kp_ang << " Ki=" << ki_ang << " Kd=" << kd_ang << "\n";
    std::cout << "========================================\n\n";
    
    PIDController linear_pid(kp_lin, ki_lin, kd_lin, 1.0f, -1.0f);
    PIDController angular_pid(kp_ang, ki_ang, kd_ang, 1.0f, -1.0f);
    
    SimulationState state;
    state.current = {0.0f, 0.0f, 0.0f};
    state.target = {5.0f, 5.0f, M_PI / 4.0f};
    
    const float dt = 0.1f; // 100ms timestep
    const int max_steps = 200;
    const float convergence_threshold = 0.1f;
    
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "Step | Pos(x,y) | Yaw | Distance | Yaw Error | Cmd(v,w)\n";
    std::cout << "-----+----------+-----+----------+-----------+----------\n";
    
    int converged_at = -1;
    
    for (int step = 0; step < max_steps; ++step) {
        DiffDriveTwist cmd = compute_pid_from_slam(
            state.current, state.target, linear_pid, angular_pid
        );
        
        simulate_step(state, cmd, dt);
        
        float dx = state.target.x - state.current.x;
        float dy = state.target.y - state.current.y;
        float distance = std::sqrt(dx * dx + dy * dy);
        float yaw_error = state.target.yaw - state.current.yaw;
        while (yaw_error > M_PI) yaw_error -= 2.0f * M_PI;
        while (yaw_error < -M_PI) yaw_error += 2.0f * M_PI;
        
        if (step % 10 == 0) {
            std::cout << std::setw(4) << step << " | "
                     << "(" << std::setw(5) << state.current.x << "," 
                     << std::setw(5) << state.current.y << ") | "
                     << std::setw(5) << state.current.yaw << " | "
                     << std::setw(8) << distance << " | "
                     << std::setw(9) << yaw_error << " | "
                     << "(" << std::setw(5) << cmd.linear_x << "," 
                     << std::setw(5) << cmd.angular_z << ")\n";
        }
        
        if (converged_at < 0 && distance < convergence_threshold && 
            std::abs(yaw_error) < convergence_threshold) {
            converged_at = step;
        }
    }
    
    float final_dx = state.target.x - state.current.x;
    float final_dy = state.target.y - state.current.y;
    float final_distance = std::sqrt(final_dx * final_dx + final_dy * final_dy);
    
    std::cout << "\nResults:\n";
    std::cout << "Final distance to target: " << final_distance << " m\n";
    std::cout << "Converged at step: " << (converged_at >= 0 ? std::to_string(converged_at) : "Did not converge") << "\n";
    
    if (final_distance > 0.5f) {
        std::cout << "WARNING: Did not reach target!\n";
    }
}

int main() {
    std::cout << "PID Controller Tuning Test\n";
    std::cout << "===========================\n";
    
    // Test different parameter sets
    std::vector<std::tuple<float, float, float, float, float, float>> test_cases = {
        // Linear (Kp, Ki, Kd), Angular (Kp, Ki, Kd)
        {0.5f, 0.0f, 0.1f,   1.0f, 0.0f, 0.2f},  // Conservative
        {1.0f, 0.1f, 0.2f,   2.0f, 0.1f, 0.3f},  // Moderate
        {2.0f, 0.2f, 0.5f,   3.0f, 0.2f, 0.5f},  // Aggressive
        {0.8f, 0.05f, 0.15f, 1.5f, 0.05f, 0.25f} // Balanced (recommended start)
    };
    
    for (const auto& params : test_cases) {
        test_pid_parameters(
            std::get<0>(params), std::get<1>(params), std::get<2>(params),
            std::get<3>(params), std::get<4>(params), std::get<5>(params)
        );
    }
    
    std::cout << "\n\nRecommendations for tuning:\n";
    std::cout << "1. Start with Kp only (Ki=0, Kd=0)\n";
    std::cout << "2. Increase Kp until you see oscillation\n";
    std::cout << "3. Add Kd to dampen oscillation\n";
    std::cout << "4. Add Ki to eliminate steady-state error\n";
    std::cout << "5. Angular gains typically need to be higher than linear\n";
    
    return 0;
}
#endif
