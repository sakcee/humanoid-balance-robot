/**
 * @file balance_monitor.cpp
 * @brief Real-time balance monitoring dashboard node
 *
 * Aggregates data from fall detection + controller and prints
 * a live terminal dashboard. Also publishes combined telemetry.
 *
 * Topics:
 *  Subscribers : /fall_detection/state   (std_msgs/String)
 *                /fall_detection/angles  (geometry_msgs/Vector3)
 *                /controller/status      (std_msgs/String)
 *                /joint_commands         (geometry_msgs/Vector3)
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <string>
#include <iomanip>
#include <sstream>

using namespace std::chrono_literals;

class BalanceMonitor : public rclcpp::Node {
public:
  BalanceMonitor() : Node("balance_monitor") {

    state_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/fall_detection/state", 10,
      [this](const std_msgs::msg::String::SharedPtr msg) { fall_state_ = msg->data; });

    angle_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
      "/fall_detection/angles", 10,
      [this](const geometry_msgs::msg::Vector3::SharedPtr msg) {
        roll_ = msg->x; pitch_ = msg->y; tilt_ = msg->z;
      });

    torque_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
      "/joint_commands", 10,
      [this](const geometry_msgs::msg::Vector3::SharedPtr msg) {
        trq_roll_ = msg->x; trq_pitch_ = msg->y;
      });

    hud_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/balance_hud", 10);

    // Dashboard timer — 2 Hz (readable in terminal)
    dashboard_timer_ = this->create_wall_timer(
      500ms, std::bind(&BalanceMonitor::printDashboard, this));

    // HUD marker timer — 10 Hz
    hud_timer_ = this->create_wall_timer(
      100ms, std::bind(&BalanceMonitor::publishHUD, this));

    RCLCPP_INFO(this->get_logger(), "Balance Monitor started");
  }

private:
  std::string fall_state_ = "STABLE";
  double roll_ = 0.0, pitch_ = 0.0, tilt_ = 0.0;
  double trq_roll_ = 0.0, trq_pitch_ = 0.0;
  int    event_count_ = 0;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr         state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr   angle_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr   torque_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr hud_marker_pub_;
  rclcpp::TimerBase::SharedPtr dashboard_timer_;
  rclcpp::TimerBase::SharedPtr hud_timer_;

  std::string colorState(const std::string & s) {
    if (s == "STABLE")   return "\033[32m" + s + "\033[0m";   // green
    if (s == "WARNING")  return "\033[33m" + s + "\033[0m";   // yellow
    if (s == "FALLING")  return "\033[91m" + s + "\033[0m";   // bright red
    if (s == "FALLEN")   return "\033[31;1m" + s + "\033[0m"; // bold red
    if (s == "RECOVERY") return "\033[36m" + s + "\033[0m";   // cyan
    return s;
  }

  std::string barChart(double val, double max_val, int width = 20) {
    int filled = (int)(std::abs(val) / max_val * width);
    filled = std::min(filled, width);
    std::string bar = "[";
    for (int i = 0; i < width; i++) bar += (i < filled ? "█" : "░");
    bar += "]";
    return bar;
  }

  void printDashboard() {
    std::ostringstream ss;
    ss << "\n";
    ss << "╔════════════════════════════════════════════╗\n";
    ss << "║   HUMANOID BALANCE MONITOR  (ROS2 Humble)  ║\n";
    ss << "╠════════════════════════════════════════════╣\n";
    ss << "║  Fall State : " << std::left << std::setw(28) << colorState(fall_state_) << " ║\n";
    ss << "╠════════════════════════════════════════════╣\n";
    ss << "║  Roll  : " << std::setw(7) << std::fixed << std::setprecision(1) << roll_
       << "°   " << barChart(roll_, 60.0) << "  ║\n";
    ss << "║  Pitch : " << std::setw(7) << pitch_
       << "°   " << barChart(pitch_, 60.0) << "  ║\n";
    ss << "║  Tilt  : " << std::setw(7) << tilt_
       << "°   " << barChart(tilt_, 60.0) << "  ║\n";
    ss << "╠════════════════════════════════════════════╣\n";
    ss << "║  Torque Roll  : " << std::setw(8) << trq_roll_  << " Nm               ║\n";
    ss << "║  Torque Pitch : " << std::setw(8) << trq_pitch_ << " Nm               ║\n";
    ss << "╚════════════════════════════════════════════╝\n";

    RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
  }

  void publishHUD() {
    visualization_msgs::msg::MarkerArray arr;

    // ── Tilt bar ──────────────────────────────────────────────────────────
    visualization_msgs::msg::Marker tilt_bar;
    tilt_bar.header.frame_id = "torso";
    tilt_bar.header.stamp    = this->now();
    tilt_bar.ns              = "hud";
    tilt_bar.id              = 10;
    tilt_bar.type            = visualization_msgs::msg::Marker::CUBE;
    tilt_bar.action          = visualization_msgs::msg::Marker::ADD;
    tilt_bar.pose.position.x = 0.35;
    tilt_bar.pose.position.y = 0.0;
    tilt_bar.pose.position.z = 0.2;
    double normalized_tilt = std::min(std::abs(tilt_) / 60.0, 1.0);
    tilt_bar.scale.x = 0.03;
    tilt_bar.scale.y = 0.03;
    tilt_bar.scale.z = 0.01 + normalized_tilt * 0.4;
    tilt_bar.color.r = (float)normalized_tilt;
    tilt_bar.color.g = 1.0f - (float)normalized_tilt;
    tilt_bar.color.b = 0.0;
    tilt_bar.color.a = 0.9;
    arr.markers.push_back(tilt_bar);

    hud_marker_pub_->publish(arr);
  }
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BalanceMonitor>());
  rclcpp::shutdown();
  return 0;
}
