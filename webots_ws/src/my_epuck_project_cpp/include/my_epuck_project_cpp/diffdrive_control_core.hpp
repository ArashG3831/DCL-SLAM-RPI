#pragma once

#include "my_epuck_project_cpp/quadrature_decoder.hpp"

#include <chrono>
#include <array>
#include <cstdint>
#include <deque>
#include <string>

namespace my_epuck_project_cpp {

constexpr double kWheelRadiusM = 0.0350;
constexpr double kWheelSeparationCmdM = 0.22235;
constexpr double kWheelSeparationOdomM = 0.22235;
constexpr double kEncoderCpr = 4606.0;
constexpr double kMaxRpmEstimate = 55.0;
constexpr double kMaxTargetRpm = 50.0;
constexpr double kControlDt = 0.05;
constexpr double kKp = 0.020;
constexpr double kKi = 0.020;
constexpr double kRpmZeroEps = 0.15;
constexpr double kMinEffectiveRpm = 12.0;

double apply_motor_deadband_rpm(double rpm);
double rpm_to_mps(double rpm, double radius = kWheelRadiusM);
double mps_to_rpm(double mps, double radius = kWheelRadiusM);

struct WheelOutput {
  double target_rpm{0.0};
  double measured_rpm{0.0};
  double logical_pwm_command{0.0};
  std::int64_t count_delta{0};
  std::uint64_t valid_delta{0};
  std::uint64_t invalid_delta{0};
  std::uint64_t a_edge_delta{0};
  std::uint64_t b_edge_delta{0};
  EncoderSnapshot encoder;
};

struct ControlOutput {
  WheelOutput left;
  WheelOutput right;
  double v{0.0};
  double omega{0.0};
  double x{0.0};
  double y{0.0};
  double theta{0.0};
  bool fault_latched{false};
  std::string fault_reason;
  std::string fault_detail;
};

class DiffDriveControlCore {
public:
  struct SafetyConfig {
    bool enabled{true};
    double window_s{0.35};
    double startup_grace_s{0.60};
    double min_target_rpm{10.0};
    double min_pwm{0.22};
    double no_pulse_timeout_s{0.35};
    int min_window_pulses{20};
    double channel_ratio_min{0.35};
    bool direction_check_enabled{false};
    int direction_min_samples{20};
    double direction_agreement_min{0.80};
    double max_encoder_rpm{90.0};
    int implausible_rpm_cycles{2};
    double underspeed_ratio{0.20};
    double underspeed_pwm{0.65};
    double underspeed_timeout_s{0.80};
  };

  DiffDriveControlCore();
  explicit DiffDriveControlCore(SafetyConfig safety);
  void configure_safety(const SafetyConfig & safety) { safety_ = safety; }
  void configure_geometry(double wheel_radius_m, double wheel_separation_cmd_m,
                          double wheel_separation_odom_m,
                          double encoder_counts_per_revolution);
  void set_command_timeout_s(double timeout_s) { command_timeout_s_ = timeout_s; }
  void set_command(double linear_mps, double angular_radps, double now_s);
  ControlOutput step(double now_s, const EncoderSnapshot & left, const EncoderSnapshot & right);
  void latch_fault(const std::string & reason, const std::string & detail);
  bool fault_latched() const { return fault_latched_; }
  void clear_fault_for_test();

private:
  struct Wheel {
    double target_rpm{0.0};
    int command_direction{0};
    double target_abs_rpm{0.0};
    double integral{0.0};
    std::int64_t last_count{0};
    std::uint64_t last_valid{0};
    std::uint64_t last_invalid{0};
    std::uint64_t last_a_edges{0};
    std::uint64_t last_b_edges{0};
    double last_time_s{0.0};
    double last_measured_rpm{0.0};
    double logical_command{0.0};
    double grace_until_s{0.0};
    double underspeed_since_s{-1.0};
    int implausible_cycles{0};
    bool pending_start{false};
    std::deque<std::array<double, 6>> health_samples;
  };

  WheelOutput update_wheel(Wheel & wheel, const EncoderSnapshot & snapshot, double now_s);
  void evaluate_safety(const ControlOutput & output, double now_s);
  double window_sum(const Wheel & wheel, int index) const;
  double age_s(std::uint64_t timestamp_ns, double now_s) const;

  SafetyConfig safety_;
  Wheel left_;
  Wheel right_;
  double command_linear_{0.0};
  double command_angular_{0.0};
  double last_command_time_s_{0.0};
  double command_timeout_s_{0.7};
  double wheel_separation_cmd_m_{kWheelSeparationCmdM};
  double wheel_separation_odom_m_{kWheelSeparationOdomM};
  double wheel_radius_m_{kWheelRadiusM};
  double encoder_cpr_{kEncoderCpr};
  double x_{0.0};
  double y_{0.0};
  double theta_{0.0};
  double last_odom_time_s_{0.0};
  bool fault_latched_{false};
  std::string fault_reason_;
  std::string fault_detail_;
};

}  // namespace my_epuck_project_cpp
