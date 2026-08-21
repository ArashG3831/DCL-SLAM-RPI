#include "my_epuck_project_cpp/diffdrive_control_core.hpp"
#include "my_epuck_project_cpp/lgpio_motor_hardware.hpp"

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <atomic>
#include <cstdio>
#include <chrono>
#include <cmath>
#include <mutex>
#include <deque>
#include <vector>
#include <algorithm>
#include <limits>
#include <string>
#include <thread>

namespace my_epuck_project_cpp {

class RealDiffDriveNodeCpp : public rclcpp::Node {
public:
  RealDiffDriveNodeCpp()
  : Node("real_diffdrive_node"), left_encoder_(0, -1), right_encoder_(0, 1), core_(make_safety())
  {
    declare_parameter("wheel_radius_m", kWheelRadiusM);
    declare_parameter("wheel_separation_cmd_m", kWheelSeparationCmdM);
    declare_parameter("wheel_separation_odom_m", kWheelSeparationOdomM);
    declare_parameter("cmd_timeout_s", 0.7);
    declare_parameter("encoder_counts_per_wheel_revolution", kEncoderCpr);
    declare_parameter("safety_enabled", true);
    declare_parameter("safety_window_s", 0.35);
    declare_parameter("safety_startup_grace_s", 0.60);
    declare_parameter("safety_min_target_rpm", 10.0);
    declare_parameter("safety_min_pwm", 0.22);
    declare_parameter("safety_no_pulse_timeout_s", 0.35);
    declare_parameter("safety_min_window_pulses", 20);
    declare_parameter("safety_channel_ratio_min", 0.35);
    declare_parameter("safety_direction_check_enabled", false);
    declare_parameter("safety_direction_min_samples", 20);
    declare_parameter("safety_direction_agreement_min", 0.80);
    declare_parameter("safety_max_encoder_rpm", 90.0);
    declare_parameter("safety_implausible_rpm_cycles", 2);
    declare_parameter("safety_underspeed_ratio", 0.20);
    declare_parameter("safety_underspeed_pwm", 0.65);
    declare_parameter("safety_underspeed_timeout_s", 0.80);

    command_timeout_s_ = get_parameter("cmd_timeout_s").as_double();
    core_.configure_geometry(
      get_parameter("wheel_radius_m").as_double(),
      get_parameter("wheel_separation_cmd_m").as_double(),
      get_parameter("wheel_separation_odom_m").as_double(),
      get_parameter("encoder_counts_per_wheel_revolution").as_double());
    core_.set_command_timeout_s(command_timeout_s_);
    core_.configure_safety(read_safety_parameters());
    hardware_ = std::make_unique<LgpioMotorHardware>(left_encoder_, right_encoder_);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    fault_pub_ = create_publisher<std_msgs::msg::String>(
      "/motor_safety/fault", rclcpp::QoS(1).reliable().transient_local());
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    diagnostic_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() { publish_diagnostics(); });
    cmd_stamped_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      "/cmd_vel", 10, [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr msg) {
        set_command(msg->twist);
      });
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel_unstamped", 10, [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
        set_command(*msg);
      });
    RCLCPP_INFO(get_logger(), "real_diffdrive_node_cpp opt-in backend started; Python remains default");
    RCLCPP_INFO(get_logger(), "Native GPIO backend: liblgpio API 0x00020200, X4, 20 Hz deadline thread");
    control_thread_ = std::thread(&RealDiffDriveNodeCpp::control_loop, this);
  }

  ~RealDiffDriveNodeCpp() override { stop(); }

private:
  static DiffDriveControlCore::SafetyConfig make_safety()
  {
    return DiffDriveControlCore::SafetyConfig{};
  }

  DiffDriveControlCore::SafetyConfig read_safety_parameters() const
  {
    DiffDriveControlCore::SafetyConfig safety;
    safety.enabled = get_parameter("safety_enabled").as_bool();
    safety.window_s = get_parameter("safety_window_s").as_double();
    safety.startup_grace_s = get_parameter("safety_startup_grace_s").as_double();
    safety.min_target_rpm = get_parameter("safety_min_target_rpm").as_double();
    safety.min_pwm = get_parameter("safety_min_pwm").as_double();
    safety.no_pulse_timeout_s = get_parameter("safety_no_pulse_timeout_s").as_double();
    safety.min_window_pulses = get_parameter("safety_min_window_pulses").as_int();
    safety.channel_ratio_min = get_parameter("safety_channel_ratio_min").as_double();
    safety.direction_check_enabled = get_parameter("safety_direction_check_enabled").as_bool();
    safety.direction_min_samples = get_parameter("safety_direction_min_samples").as_int();
    safety.direction_agreement_min = get_parameter("safety_direction_agreement_min").as_double();
    safety.max_encoder_rpm = get_parameter("safety_max_encoder_rpm").as_double();
    safety.implausible_rpm_cycles = get_parameter("safety_implausible_rpm_cycles").as_int();
    safety.underspeed_ratio = get_parameter("safety_underspeed_ratio").as_double();
    safety.underspeed_pwm = get_parameter("safety_underspeed_pwm").as_double();
    safety.underspeed_timeout_s = get_parameter("safety_underspeed_timeout_s").as_double();
    return safety;
  }

  void set_command(const geometry_msgs::msg::Twist & msg)
  {
    {
      std::lock_guard<std::mutex> lock(core_mutex_);
      if (core_.fault_latched()) return;
    }
    std::lock_guard<std::mutex> lock(command_mutex_);
    command_linear_ = msg.linear.x;
    command_angular_ = msg.angular.z;
    command_time_s_ = steady_seconds();
  }

  static double steady_seconds()
  {
    return std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  void control_loop()
  {
    double next_deadline = steady_seconds();
    double previous_start = 0.0;
    while (!stop_requested_.load()) {
      const double now = steady_seconds();
      if (next_deadline > now) {
        std::this_thread::sleep_for(std::chrono::duration<double>(next_deadline - now));
      }
      if (stop_requested_.load()) break;
      const double scheduled_start = next_deadline;
      const double actual_start = steady_seconds();
      if (previous_start > 0.0) {
        std::lock_guard<std::mutex> lock(timing_mutex_);
        control_periods_.push_back(actual_start - previous_start);
        if (control_periods_.size() > 8192) control_periods_.pop_front();
      }
      previous_start = actual_start;
      if (actual_start - scheduled_start > 0.001) ++deadline_misses_;
      double linear, angular, command_time;
      {
        std::lock_guard<std::mutex> lock(command_mutex_);
        linear = command_linear_; angular = command_angular_; command_time = command_time_s_;
      }
      const double t = steady_seconds();
      if (t - command_time > command_timeout_s_) { linear = 0.0; angular = 0.0; }
      ControlOutput output;
      {
        std::lock_guard<std::mutex> lock(core_mutex_);
        core_.set_command(linear, angular, t);
        output = core_.step(t, left_encoder_.snapshot(), right_encoder_.snapshot());
      }
      const double actual_end = steady_seconds();
      {
        std::lock_guard<std::mutex> lock(timing_mutex_);
        control_exec_times_.push_back(actual_end - actual_start);
        if (control_exec_times_.size() > 8192) control_exec_times_.pop_front();
      }
      update_encoder_age(output, t);
      hardware_->set_motor(output.left.logical_pwm_command, -1, 13, 5, 6);
      hardware_->set_motor(output.right.logical_pwm_command, 1, 18, 23, 24);
      publish_output(output);
      if (output.fault_latched && !fault_reported_) {
        std_msgs::msg::String msg;
        msg.data = output.fault_reason + ": " + output.fault_detail;
        fault_pub_->publish(msg);
        RCLCPP_FATAL(get_logger(), "MOTOR SAFETY FAULT LATCHED: %s", msg.data.c_str());
        fault_reported_ = true;
      }
      next_deadline += kControlDt;
      const double after = steady_seconds();
      if (next_deadline <= after) {
        const auto skipped = static_cast<int>((after - next_deadline) / kControlDt) + 1;
        next_deadline += skipped * kControlDt;
      }
    }
    hardware_->stop_all();
  }

  void update_encoder_age(const ControlOutput & output, double now_s)
  {
    const auto age = [now_s](std::uint64_t stamp) {
      if (stamp == 0) return std::numeric_limits<double>::infinity();
      return std::max(0.0, now_s - static_cast<double>(stamp) / 1e9);
    };
    const double left_a_age = age(output.left.encoder.last_a_edge_time_ns);
    const double left_b_age = age(output.left.encoder.last_b_edge_time_ns);
    const double right_a_age = age(output.right.encoder.last_a_edge_time_ns);
    const double right_b_age = age(output.right.encoder.last_b_edge_time_ns);
    const double left_age = std::max(left_a_age, left_b_age);
    const double right_age = std::max(right_a_age, right_b_age);
    std::lock_guard<std::mutex> lock(timing_mutex_);
    if (std::isfinite(left_age)) {
      encoder_ages_.push_back(left_age);
      if (encoder_ages_.size() > 8192) encoder_ages_.pop_front();
    } else {
      ++unknown_encoder_age_cycles_;
    }
    if (std::isfinite(right_age)) {
      encoder_ages_.push_back(right_age);
      if (encoder_ages_.size() > 8192) encoder_ages_.pop_front();
    } else {
      ++unknown_encoder_age_cycles_;
    }
  }

  static double percentile(std::deque<double> values, double p)
  {
    if (values.empty()) return 0.0;
    std::sort(values.begin(), values.end());
    const auto index = static_cast<std::size_t>(std::llround(
      static_cast<double>(values.size() - 1) * p / 100.0));
    return values[std::min(index, values.size() - 1)];
  }

  void publish_diagnostics()
  {
    std::deque<double> periods, execs, ages;
    std::size_t misses, unknown;
    const auto left = left_encoder_.snapshot();
    const auto right = right_encoder_.snapshot();
    {
      std::lock_guard<std::mutex> lock(timing_mutex_);
      periods = control_periods_;
      execs = control_exec_times_;
      ages = encoder_ages_;
      misses = deadline_misses_;
      unknown = unknown_encoder_age_cycles_;
    }
    const auto max_or_zero = [](const std::deque<double> & values) {
      return values.empty() ? 0.0 : *std::max_element(values.begin(), values.end());
    };
    RCLCPP_INFO(get_logger(),
      "MOTOR_CPP timing period_ms[p50=%.3f p95=%.3f p99=%.3f max=%.3f] "
      "exec_ms[p50=%.3f p95=%.3f p99=%.3f max=%.3f] deadline_misses=%zu "
      "encoder_age_ms[max=%.3f] unknown_age_cycles=%zu "
      "L[count=%lld valid=%llu invalid=%llu A=%llu B=%llu] "
      "R[count=%lld valid=%llu invalid=%llu A=%llu B=%llu]",
      percentile(periods, 50.0) * 1000.0, percentile(periods, 95.0) * 1000.0,
      percentile(periods, 99.0) * 1000.0, max_or_zero(periods) * 1000.0,
      percentile(execs, 50.0) * 1000.0, percentile(execs, 95.0) * 1000.0,
      percentile(execs, 99.0) * 1000.0, max_or_zero(execs) * 1000.0, misses,
      max_or_zero(ages) * 1000.0, unknown,
      static_cast<long long>(left.count),
      static_cast<unsigned long long>(left.valid_transition_count),
      static_cast<unsigned long long>(left.invalid_transition_count),
      static_cast<unsigned long long>(left.a_edge_count),
      static_cast<unsigned long long>(left.b_edge_count),
      static_cast<long long>(right.count),
      static_cast<unsigned long long>(right.valid_transition_count),
      static_cast<unsigned long long>(right.invalid_transition_count),
      static_cast<unsigned long long>(right.a_edge_count),
      static_cast<unsigned long long>(right.b_edge_count));
  }

  void publish_output(const ControlOutput & output)
  {
    const auto stamp = get_clock()->now();
    nav_msgs::msg::Odometry msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "odom";
    msg.child_frame_id = "base_link";
    msg.pose.pose.position.x = output.x;
    msg.pose.pose.position.y = output.y;
    msg.pose.pose.orientation.z = std::sin(output.theta / 2.0);
    msg.pose.pose.orientation.w = std::cos(output.theta / 2.0);
    msg.twist.twist.linear.x = output.v;
    msg.twist.twist.angular.z = output.omega;
    msg.pose.covariance[0] = 0.02; msg.pose.covariance[7] = 0.02; msg.pose.covariance[35] = 0.10;
    msg.twist.covariance[0] = 0.05; msg.twist.covariance[35] = 0.10;
    odom_pub_->publish(msg);

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = "odom";
    tf.child_frame_id = "base_link";
    tf.transform.translation.x = output.x;
    tf.transform.translation.y = output.y;
    tf.transform.rotation.z = msg.pose.pose.orientation.z;
    tf.transform.rotation.w = msg.pose.pose.orientation.w;
    tf_broadcaster_->sendTransform(tf);
  }

  void stop()
  {
    if (stop_requested_.exchange(true)) return;
    if (control_thread_.joinable()) control_thread_.join();
    if (hardware_) hardware_->stop_all();
  }

  QuadratureDecoder left_encoder_;
  QuadratureDecoder right_encoder_;
  DiffDriveControlCore core_;
  std::unique_ptr<LgpioMotorHardware> hardware_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr fault_pub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_stamped_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr diagnostic_timer_;
  std::thread control_thread_;
  std::atomic<bool> stop_requested_{false};
  std::mutex command_mutex_;
  std::mutex core_mutex_;
  double command_linear_{0.0};
  double command_angular_{0.0};
  double command_time_s_{0.0};
  double command_timeout_s_{0.7};
  bool fault_reported_{false};
  std::mutex timing_mutex_;
  std::deque<double> control_periods_;
  std::deque<double> control_exec_times_;
  std::deque<double> encoder_ages_;
  std::size_t deadline_misses_{0};
  std::size_t unknown_encoder_age_cycles_{0};
};

}  // namespace my_epuck_project_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<my_epuck_project_cpp::RealDiffDriveNodeCpp>();
    rclcpp::spin(node);
  } catch (const std::exception & error) {
    fprintf(stderr, "real_diffdrive_node_cpp: %s\n", error.what());
  }
  if (rclcpp::ok()) rclcpp::shutdown();
  return 0;
}
