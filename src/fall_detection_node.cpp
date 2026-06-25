/**
 * @file fall_detection_node.cpp
 * @brief Humanoid Upper Body — Fall Detection System
 *
 * Algorithm:
 *  1. Subscribes to dual IMU data (torso + head) at 200 Hz
 *  2. Applies a Complementary Filter to fuse accelerometer & gyroscope data
 *  3. Computes roll/pitch angles of the torso
 *  4. Uses a multi-threshold state machine:
 *       STABLE → WARNING → FALLING → FALLEN → RECOVERY
 *  5. Publishes fall state, tilt angles, and visual markers to RViz
 *
 * Topics:
 *  Subscribers : /imu/torso  (sensor_msgs/Imu)
 *                /imu/head   (sensor_msgs/Imu)
 *  Publishers  : /fall_detection/state   (std_msgs/String)
 *                /fall_detection/angles  (geometry_msgs/Vector3)
 *                /fall_detection/markers (visualization_msgs/MarkerArray)
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <cmath>
#include <deque>
#include <chrono>
#include <string>
#include <memory>

using namespace std::chrono_literals;

// ─── Fall Detection Thresholds ────────────────────────────────────────────────
constexpr double WARNING_TILT_DEG  = 20.0;   // deg — start warning
constexpr double FALLING_TILT_DEG  = 35.0;   // deg — falling state
constexpr double FALLEN_TILT_DEG   = 60.0;   // deg — fully fallen
constexpr double ANGULAR_VEL_THRESH = 1.5;   // rad/s — rapid tilt
constexpr double IMPACT_ACCEL_THRESH = 25.0; // m/s² — impact detection
constexpr double COMPLEMENTARY_ALPHA = 0.98; // filter coefficient
constexpr int    SMOOTHING_WINDOW    = 10;   // samples for moving average

// ─── State Machine ────────────────────────────────────────────────────────────
enum class FallState {
  STABLE,
  WARNING,
  FALLING,
  FALLEN,
  RECOVERY
};

std::string stateToString(FallState s) {
  switch (s) {
    case FallState::STABLE:   return "STABLE";
    case FallState::WARNING:  return "WARNING";
    case FallState::FALLING:  return "FALLING";
    case FallState::FALLEN:   return "FALLEN";
    case FallState::RECOVERY: return "RECOVERY";
    default:                  return "UNKNOWN";
  }
}

// ─── Node ─────────────────────────────────────────────────────────────────────
class FallDetectionNode : public rclcpp::Node {
public:
  FallDetectionNode() : Node("fall_detection_node") {

    // Parameters
    this->declare_parameter("warning_tilt_deg",   WARNING_TILT_DEG);
    this->declare_parameter("falling_tilt_deg",   FALLING_TILT_DEG);
    this->declare_parameter("fallen_tilt_deg",    FALLEN_TILT_DEG);
    this->declare_parameter("angular_vel_thresh", ANGULAR_VEL_THRESH);
    this->declare_parameter("impact_accel_thresh",IMPACT_ACCEL_THRESH);
    this->declare_parameter("complementary_alpha",COMPLEMENTARY_ALPHA);

    load_parameters();

    // Subscribers
    imu_torso_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu/torso", 10,
      std::bind(&FallDetectionNode::imuTorsoCallback, this, std::placeholders::_1));

    imu_head_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu/head", 10,
      std::bind(&FallDetectionNode::imuHeadCallback, this, std::placeholders::_1));

    // Publishers
    state_pub_  = this->create_publisher<std_msgs::msg::String>(
      "/fall_detection/state", 10);

    angle_pub_  = this->create_publisher<geometry_msgs::msg::Vector3>(
      "/fall_detection/angles", 10);

    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/fall_detection/markers", 10);

    // Periodic status log
    log_timer_ = this->create_wall_timer(
      1000ms, std::bind(&FallDetectionNode::logStatus, this));

    RCLCPP_INFO(this->get_logger(),
      "Fall Detection Node initialized | Thresholds: warn=%.1f° fall=%.1f° fallen=%.1f°",
      warning_tilt_deg_, falling_tilt_deg_, fallen_tilt_deg_);
  }

private:
  // ── Parameters
  double warning_tilt_deg_, falling_tilt_deg_, fallen_tilt_deg_;
  double angular_vel_thresh_, impact_accel_thresh_, alpha_;

  // ── Filter state
  double roll_  = 0.0;
  double pitch_ = 0.0;
  double last_time_ = -1.0;
  bool   first_imu_ = true;

  // ── Smoothing buffers
  std::deque<double> roll_buf_, pitch_buf_;

  // ── State machine
  FallState current_state_ = FallState::STABLE;
  int       stable_count_  = 0;
  static constexpr int STABLE_RECOVERY_COUNT = 50;

  // ── ROS handles
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_torso_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_head_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr      state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr angle_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr log_timer_;

  // ─────────────────────────────────────────────────────────────────────────
  void load_parameters() {
    warning_tilt_deg_    = this->get_parameter("warning_tilt_deg").as_double();
    falling_tilt_deg_    = this->get_parameter("falling_tilt_deg").as_double();
    fallen_tilt_deg_     = this->get_parameter("fallen_tilt_deg").as_double();
    angular_vel_thresh_  = this->get_parameter("angular_vel_thresh").as_double();
    impact_accel_thresh_ = this->get_parameter("impact_accel_thresh").as_double();
    alpha_               = this->get_parameter("complementary_alpha").as_double();
  }

  // ─────────────────────────────────────────────────────────────────────────
  void imuTorsoCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {

    double t = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    double dt = (first_imu_) ? 0.005 : (t - last_time_);
    last_time_ = t;
    first_imu_ = false;

    // Clamp dt to sane range
    if (dt <= 0.0 || dt > 0.1) dt = 0.005;

    // Raw accel
    double ax = msg->linear_acceleration.x;
    double ay = msg->linear_acceleration.y;
    double az = msg->linear_acceleration.z;

    // Raw gyro
    double gx = msg->angular_velocity.x;
    double gy = msg->angular_velocity.y;

    // ── Accelerometer angles (noisy, drift-free long-term)
    double accel_roll  = std::atan2(ay, az);
    double accel_pitch = std::atan2(-ax, std::sqrt(ay*ay + az*az));

    // ── Complementary Filter: fuse gyro (fast) + accel (slow)
    roll_  = alpha_ * (roll_  + gx * dt) + (1.0 - alpha_) * accel_roll;
    pitch_ = alpha_ * (pitch_ + gy * dt) + (1.0 - alpha_) * accel_pitch;

    // ── Moving average smoothing
    double roll_deg  = roll_  * 180.0 / M_PI;
    double pitch_deg = pitch_ * 180.0 / M_PI;

    roll_buf_.push_back(roll_deg);
    pitch_buf_.push_back(pitch_deg);
    if ((int)roll_buf_.size()  > SMOOTHING_WINDOW) roll_buf_.pop_front();
    if ((int)pitch_buf_.size() > SMOOTHING_WINDOW) pitch_buf_.pop_front();

    double smooth_roll  = movingAverage(roll_buf_);
    double smooth_pitch = movingAverage(pitch_buf_);

    // ── Total angular velocity magnitude
    double omega = std::sqrt(gx*gx + gy*gy +
      msg->angular_velocity.z * msg->angular_velocity.z);

    // ── Total acceleration magnitude (detect impact)
    double accel_mag = std::sqrt(ax*ax + ay*ay + az*az);

    // ── Tilt magnitude (worst axis)
    double tilt = std::max(std::abs(smooth_roll), std::abs(smooth_pitch));

    // ── State machine update
    updateStateMachine(tilt, omega, accel_mag);

    // ── Publish
    publishState();
    publishAngles(smooth_roll, smooth_pitch, tilt);
    publishMarkers(smooth_roll, smooth_pitch);
  }

  // ─────────────────────────────────────────────────────────────────────────
  void imuHeadCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    // Head IMU used for secondary verification — large head-torso angle delta
    // indicates aggressive disturbance (whiplash-like motion)
    double head_omega = std::sqrt(
      std::pow(msg->angular_velocity.x, 2) +
      std::pow(msg->angular_velocity.y, 2) +
      std::pow(msg->angular_velocity.z, 2));

    if (head_omega > angular_vel_thresh_ * 2.5 &&
        current_state_ == FallState::STABLE) {
      RCLCPP_WARN(this->get_logger(),
        "Head IMU: High angular velocity detected (%.2f rad/s) — possible disturbance",
        head_omega);
    }
  }

  // ─────────────────────────────────────────────────────────────────────────
  void updateStateMachine(double tilt_deg, double omega, double accel_mag) {

    FallState prev = current_state_;

    switch (current_state_) {
      case FallState::STABLE:
        if (tilt_deg > falling_tilt_deg_ || omega > angular_vel_thresh_ * 2.0)
          current_state_ = FallState::FALLING;
        else if (tilt_deg > warning_tilt_deg_ || omega > angular_vel_thresh_)
          current_state_ = FallState::WARNING;
        break;

      case FallState::WARNING:
        if (tilt_deg > falling_tilt_deg_)
          current_state_ = FallState::FALLING;
        else if (tilt_deg < warning_tilt_deg_ * 0.8)
          current_state_ = FallState::STABLE;
        break;

      case FallState::FALLING:
        if (tilt_deg > fallen_tilt_deg_ || accel_mag > impact_accel_thresh_)
          current_state_ = FallState::FALLEN;
        else if (tilt_deg < warning_tilt_deg_)
          current_state_ = FallState::RECOVERY;
        break;

      case FallState::FALLEN:
        // Recovery requires sustained low tilt
        if (tilt_deg < warning_tilt_deg_) {
          stable_count_++;
          if (stable_count_ >= STABLE_RECOVERY_COUNT) {
            current_state_ = FallState::RECOVERY;
            stable_count_  = 0;
          }
        } else {
          stable_count_ = 0;
        }
        break;

      case FallState::RECOVERY:
        if (tilt_deg < warning_tilt_deg_ * 0.5)
          current_state_ = FallState::STABLE;
        else if (tilt_deg > falling_tilt_deg_)
          current_state_ = FallState::FALLING;
        break;
    }

    if (current_state_ != prev) {
      RCLCPP_WARN(this->get_logger(),
        "Fall State: %s → %s  |  tilt=%.1f°  omega=%.2f rad/s  accel=%.1f m/s²",
        stateToString(prev).c_str(),
        stateToString(current_state_).c_str(),
        tilt_deg, omega, accel_mag);
    }
  }

  // ─────────────────────────────────────────────────────────────────────────
  void publishState() {
    auto msg = std_msgs::msg::String();
    msg.data = stateToString(current_state_);
    state_pub_->publish(msg);
  }

  void publishAngles(double roll, double pitch, double tilt) {
    auto msg = geometry_msgs::msg::Vector3();
    msg.x = roll;
    msg.y = pitch;
    msg.z = tilt;   // z encodes total tilt for convenience
    angle_pub_->publish(msg);
  }

  // ─────────────────────────────────────────────────────────────────────────
  void publishMarkers(double roll_deg, double pitch_deg) {
    visualization_msgs::msg::MarkerArray arr;

    // ── Marker 0: State text above head
    visualization_msgs::msg::Marker text_marker;
    text_marker.header.frame_id = "torso";
    text_marker.header.stamp    = this->now();
    text_marker.ns              = "fall_detection";
    text_marker.id              = 0;
    text_marker.type            = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker.action          = visualization_msgs::msg::Marker::ADD;
    text_marker.pose.position.x = 0.0;
    text_marker.pose.position.y = 0.0;
    text_marker.pose.position.z = 0.70;
    text_marker.scale.z         = 0.08;
    text_marker.text            = "State: " + stateToString(current_state_) +
                                  "\nRoll: "  + std::to_string((int)roll_deg)  + "°" +
                                  "  Pitch: " + std::to_string((int)pitch_deg) + "°";

    // Color by state
    switch (current_state_) {
      case FallState::STABLE:
        setColor(text_marker, 0.0, 1.0, 0.0, 1.0); break;  // Green
      case FallState::WARNING:
        setColor(text_marker, 1.0, 0.8, 0.0, 1.0); break;  // Yellow
      case FallState::FALLING:
        setColor(text_marker, 1.0, 0.4, 0.0, 1.0); break;  // Orange
      case FallState::FALLEN:
        setColor(text_marker, 1.0, 0.0, 0.0, 1.0); break;  // Red
      case FallState::RECOVERY:
        setColor(text_marker, 0.0, 0.6, 1.0, 1.0); break;  // Blue
    }
    arr.markers.push_back(text_marker);

    // ── Marker 1: Sphere at torso center (state indicator)
    visualization_msgs::msg::Marker sphere;
    sphere.header.frame_id = "torso";
    sphere.header.stamp    = this->now();
    sphere.ns              = "fall_detection";
    sphere.id              = 1;
    sphere.type            = visualization_msgs::msg::Marker::SPHERE;
    sphere.action          = visualization_msgs::msg::Marker::ADD;
    sphere.pose.position.x = 0.0;
    sphere.pose.position.y = 0.0;
    sphere.pose.position.z = 0.20;
    sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.06;
    sphere.color   = text_marker.color;
    arr.markers.push_back(sphere);

    marker_pub_->publish(arr);
  }

  // ─────────────────────────────────────────────────────────────────────────
  void setColor(visualization_msgs::msg::Marker & m,
                float r, float g, float b, float a) {
    m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = a;
  }

  double movingAverage(const std::deque<double> & buf) {
    if (buf.empty()) return 0.0;
    double sum = 0.0;
    for (auto v : buf) sum += v;
    return sum / buf.size();
  }

  void logStatus() {
    RCLCPP_INFO(this->get_logger(),
      "[FallDetect] State=%-8s | Roll=%.1f°  Pitch=%.1f°",
      stateToString(current_state_).c_str(),
      roll_ * 180.0 / M_PI,
      pitch_ * 180.0 / M_PI);
  }
};

// ─── Main ─────────────────────────────────────────────────────────────────────
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FallDetectionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
