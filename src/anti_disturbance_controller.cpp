/**
 * @file anti_disturbance_controller.cpp
 * @brief Humanoid Upper Body — Anti-Disturbance Controller
 *
 * Control Strategy:
 *  - Primary:   PID controller on torso roll/pitch error
 *  - Secondary: Feed-forward disturbance rejection using angular acceleration
 *  - Safety:    Hard joint limits, torque saturation, emergency stop on FALLEN state
 *
 * Topics:
 *  Subscribers : /imu/torso               (sensor_msgs/Imu)
 *                /fall_detection/state    (std_msgs/String)
 *                /fall_detection/angles   (geometry_msgs/Vector3)
 *  Publishers  : /joint_commands          (geometry_msgs/Vector3)  roll/pitch/yaw torques
 *                /controller/status       (std_msgs/String)
 *                /controller/markers      (visualization_msgs/MarkerArray)
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <cmath>
#include <deque>
#include <string>
#include <memory>
#include <algorithm>

using namespace std::chrono_literals;

// ─── PID Gains (tuned for 8 kg torso, ±40° range) ────────────────────────────
struct PIDGains {
  double kp, ki, kd;
};

constexpr PIDGains ROLL_GAINS  = {25.0, 0.8, 4.5};
constexpr PIDGains PITCH_GAINS = {22.0, 0.6, 4.0};

// ─── Safety Limits ────────────────────────────────────────────────────────────
constexpr double MAX_TORQUE_NM       = 45.0;    // Nm — per axis
constexpr double INTEGRATOR_WINDUP   = 15.0;    // Nm — anti-windup clamp
constexpr double DEADBAND_DEG        = 1.5;     // deg — ignore tiny errors
constexpr double FEED_FORWARD_GAIN   = 8.0;     // disturbance rejection gain

// ─── PID Controller Class ─────────────────────────────────────────────────────
class PIDController {
public:
  PIDController(PIDGains gains) : kp_(gains.kp), ki_(gains.ki), kd_(gains.kd) {}

  double compute(double setpoint, double measurement, double dt) {
    if (dt <= 0.0 || dt > 0.1) dt = 0.005;

    double error = setpoint - measurement;

    // Deadband
    if (std::abs(error * 180.0 / M_PI) < DEADBAND_DEG) {
      integral_ = integral_ * 0.95;  // bleed integrator in deadband
      return 0.0;
    }

    // Proportional
    double p_term = kp_ * error;

    // Integral with anti-windup
    integral_ += error * dt;
    integral_  = std::clamp(integral_, -INTEGRATOR_WINDUP, INTEGRATOR_WINDUP);
    double i_term = ki_ * integral_;

    // Derivative (with low-pass filter to reduce noise)
    double raw_derivative = (error - prev_error_) / dt;
    derivative_filtered_ = 0.8 * derivative_filtered_ + 0.2 * raw_derivative;
    double d_term = kd_ * derivative_filtered_;

    prev_error_ = error;

    double output = p_term + i_term + d_term;
    return std::clamp(output, -MAX_TORQUE_NM, MAX_TORQUE_NM);
  }

  void reset() {
    integral_ = 0.0;
    prev_error_ = 0.0;
    derivative_filtered_ = 0.0;
  }

  double getIntegral() const { return integral_; }

private:
  double kp_, ki_, kd_;
  double integral_           = 0.0;
  double prev_error_         = 0.0;
  double derivative_filtered_= 0.0;
};

// ─── Node ─────────────────────────────────────────────────────────────────────
class AntiDisturbanceController : public rclcpp::Node {
public:
  AntiDisturbanceController()
  : Node("anti_disturbance_controller"),
    roll_pid_(ROLL_GAINS),
    pitch_pid_(PITCH_GAINS)
  {
    // Subscribers
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu/torso", 10,
      std::bind(&AntiDisturbanceController::imuCallback, this, std::placeholders::_1));

    fall_state_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/fall_detection/state", 10,
      std::bind(&AntiDisturbanceController::fallStateCallback, this, std::placeholders::_1));

    angle_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
      "/fall_detection/angles", 10,
      std::bind(&AntiDisturbanceController::angleCallback, this, std::placeholders::_1));

    // Publishers
    cmd_pub_    = this->create_publisher<geometry_msgs::msg::Vector3>("/joint_commands", 10);
    status_pub_ = this->create_publisher<std_msgs::msg::String>("/controller/status", 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/controller/markers", 10);

    // Log timer
    log_timer_ = this->create_wall_timer(
      500ms, std::bind(&AntiDisturbanceController::logStatus, this));

    RCLCPP_INFO(this->get_logger(),
      "Anti-Disturbance Controller ready | PID Roll(Kp=%.1f Ki=%.2f Kd=%.1f) "
      "Pitch(Kp=%.1f Ki=%.2f Kd=%.1f)",
      ROLL_GAINS.kp,  ROLL_GAINS.ki,  ROLL_GAINS.kd,
      PITCH_GAINS.kp, PITCH_GAINS.ki, PITCH_GAINS.kd);
  }

private:
  PIDController roll_pid_;
  PIDController pitch_pid_;

  std::string fall_state_  = "STABLE";
  double current_roll_deg_ = 0.0;
  double current_pitch_deg_= 0.0;
  double prev_imu_time_    = -1.0;
  double torque_roll_      = 0.0;
  double torque_pitch_     = 0.0;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr   imu_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr   fall_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr angle_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr    cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr          status_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr                                 log_timer_;

  // ─────────────────────────────────────────────────────────────────────────
  void fallStateCallback(const std_msgs::msg::String::SharedPtr msg) {
    fall_state_ = msg->data;

    // Emergency: reset integrators on state change to avoid torque spike
    if (fall_state_ == "FALLEN") {
      roll_pid_.reset();
      pitch_pid_.reset();
      RCLCPP_ERROR(this->get_logger(), "FALLEN detected — PID reset, awaiting recovery");
    }
  }

  void angleCallback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
    current_roll_deg_  = msg->x;
    current_pitch_deg_ = msg->y;
  }

  // ─────────────────────────────────────────────────────────────────────────
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {

    double t  = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    double dt = (prev_imu_time_ < 0) ? 0.005 : (t - prev_imu_time_);
    prev_imu_time_ = t;

    // ── Safety gate: no control if FALLEN ──────────────────────────────────
    if (fall_state_ == "FALLEN") {
      publishCommand(0.0, 0.0, 0.0);
      publishStatus("EMERGENCY_STOP");
      return;
    }

    // ── Convert angles to radians for PID ──────────────────────────────────
    double roll_rad  = current_roll_deg_  * M_PI / 180.0;
    double pitch_rad = current_pitch_deg_ * M_PI / 180.0;

    // ── PID control: setpoint = upright (0 rad) ────────────────────────────
    torque_roll_  = roll_pid_.compute(0.0, roll_rad,  dt);
    torque_pitch_ = pitch_pid_.compute(0.0, pitch_rad, dt);

    // ── Feed-forward disturbance rejection ─────────────────────────────────
    // Angular acceleration = derivative of angular velocity (finite difference)
    double ff_roll  = -FEED_FORWARD_GAIN * msg->angular_velocity.x;
    double ff_pitch = -FEED_FORWARD_GAIN * msg->angular_velocity.y;

    // Blend: stronger FF when FALLING, weaker when STABLE
    double ff_weight = (fall_state_ == "FALLING" || fall_state_ == "WARNING") ? 0.6 : 0.3;
    torque_roll_  += ff_weight * ff_roll;
    torque_pitch_ += ff_weight * ff_pitch;

    // ── Final clamp ────────────────────────────────────────────────────────
    torque_roll_  = std::clamp(torque_roll_,  -MAX_TORQUE_NM, MAX_TORQUE_NM);
    torque_pitch_ = std::clamp(torque_pitch_, -MAX_TORQUE_NM, MAX_TORQUE_NM);

    publishCommand(torque_roll_, torque_pitch_, 0.0);
    publishStatus("ACTIVE");
    publishMarkers();
  }

  // ─────────────────────────────────────────────────────────────────────────
  void publishCommand(double roll_torque, double pitch_torque, double yaw_torque) {
    auto cmd = geometry_msgs::msg::Vector3();
    cmd.x = roll_torque;
    cmd.y = pitch_torque;
    cmd.z = yaw_torque;
    cmd_pub_->publish(cmd);
  }

  void publishStatus(const std::string & status) {
    auto msg = std_msgs::msg::String();
    msg.data = status + " | FallState=" + fall_state_ +
               " | Roll=" + std::to_string((int)current_roll_deg_) + "deg" +
               " | Trq_R=" + std::to_string((int)torque_roll_)  + "Nm" +
               " | Trq_P=" + std::to_string((int)torque_pitch_) + "Nm";
    status_pub_->publish(msg);
  }

  void publishMarkers() {
    visualization_msgs::msg::MarkerArray arr;

    // Arrow showing correction direction (roll)
    visualization_msgs::msg::Marker arrow;
    arrow.header.frame_id = "torso";
    arrow.header.stamp    = this->now();
    arrow.ns              = "controller";
    arrow.id              = 0;
    arrow.type            = visualization_msgs::msg::Marker::ARROW;
    arrow.action          = visualization_msgs::msg::Marker::ADD;
    arrow.scale.x = 0.02;   // shaft diameter
    arrow.scale.y = 0.04;   // head diameter
    arrow.scale.z = 0.04;   // head length

    // Arrow length proportional to torque
    double normalized = std::abs(torque_roll_) / MAX_TORQUE_NM;
    geometry_msgs::msg::Point start, end;
    start.x = 0; start.y = 0; start.z = 0.2;
    end.x   = 0;
    end.y   = (torque_roll_ > 0 ? 1.0 : -1.0) * normalized * 0.3;
    end.z   = 0.2;
    arrow.points.push_back(start);
    arrow.points.push_back(end);

    // Color: green=low effort, red=high effort
    arrow.color.r = normalized;
    arrow.color.g = 1.0f - (float)normalized;
    arrow.color.b = 0.0;
    arrow.color.a = 0.85;
    arr.markers.push_back(arrow);

    marker_pub_->publish(arr);
  }

  void logStatus() {
    RCLCPP_INFO(this->get_logger(),
      "[Controller] State=%-8s | Roll=%+6.1f°  Pitch=%+6.1f° | "
      "Torque R=%+6.1fNm  P=%+6.1fNm",
      fall_state_.c_str(),
      current_roll_deg_, current_pitch_deg_,
      torque_roll_, torque_pitch_);
  }
};

// ─── Main ─────────────────────────────────────────────────────────────────────
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AntiDisturbanceController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
