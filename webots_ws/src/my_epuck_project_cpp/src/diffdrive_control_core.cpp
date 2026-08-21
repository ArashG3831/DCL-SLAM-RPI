#include "my_epuck_project_cpp/diffdrive_control_core.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <vector>

namespace my_epuck_project_cpp {

constexpr double kPi = 3.14159265358979323846;

double apply_motor_deadband_rpm(double rpm)
{
  if (std::abs(rpm) < kRpmZeroEps) return 0.0;
  if (std::abs(rpm) < kMinEffectiveRpm) return std::copysign(kMinEffectiveRpm, rpm);
  return rpm;
}

double rpm_to_mps(double rpm, double radius)
{
  return rpm / 60.0 * (2.0 * kPi * radius);
}

double mps_to_rpm(double mps, double radius)
{
  return mps / (2.0 * kPi * radius) * 60.0;
}

DiffDriveControlCore::DiffDriveControlCore()
: safety_{} {}

DiffDriveControlCore::DiffDriveControlCore(SafetyConfig safety)
: safety_(safety) {}

void DiffDriveControlCore::configure_geometry(double wheel_radius_m,
                                              double wheel_separation_cmd_m,
                                              double wheel_separation_odom_m,
                                              double encoder_counts_per_revolution)
{
  wheel_radius_m_ = wheel_radius_m;
  wheel_separation_cmd_m_ = wheel_separation_cmd_m;
  wheel_separation_odom_m_ = wheel_separation_odom_m;
  encoder_cpr_ = encoder_counts_per_revolution;
}

void DiffDriveControlCore::set_command(double linear_mps, double angular_radps, double now_s)
{
  command_linear_ = linear_mps;
  command_angular_ = angular_radps;
  last_command_time_s_ = now_s;
  const double left_mps = linear_mps - angular_radps * wheel_separation_cmd_m_ / 2.0;
  const double right_mps = linear_mps + angular_radps * wheel_separation_cmd_m_ / 2.0;
  const double left_target = apply_motor_deadband_rpm(mps_to_rpm(left_mps, wheel_radius_m_));
  const double right_target = apply_motor_deadband_rpm(mps_to_rpm(right_mps, wheel_radius_m_));

  auto set_wheel = [&](Wheel & wheel, double target) {
    target = std::clamp(target, -kMaxTargetRpm, kMaxTargetRpm);
    const int old_direction = wheel.command_direction;
    const int new_direction = target > 0.0 ? 1 : target < 0.0 ? -1 : 0;
    if (std::abs(target - wheel.target_rpm) < 1e-6) return;
    wheel.target_rpm = target;
    wheel.command_direction = new_direction;
    wheel.target_abs_rpm = std::abs(target);
    if (new_direction == 0) {
      wheel.integral = 0.0;
      wheel.health_samples.clear();
      wheel.underspeed_since_s = -1.0;
      wheel.implausible_cycles = 0;
      wheel.logical_command = 0.0;
      wheel.pending_start = false;
      return;
    }
    if (old_direction == 0 || old_direction != new_direction) {
      wheel.integral = 0.0;
      wheel.health_samples.clear();
      wheel.underspeed_since_s = -1.0;
      wheel.implausible_cycles = 0;
      wheel.grace_until_s = now_s + safety_.startup_grace_s;
      // The Python implementation applies this first command in update().
      wheel.logical_command = new_direction *
        std::clamp(wheel.target_abs_rpm / kMaxRpmEstimate, 0.25, 1.0);
      wheel.pending_start = true;
    }
  };
  set_wheel(left_, left_target);
  set_wheel(right_, right_target);
}

WheelOutput DiffDriveControlCore::update_wheel(
  Wheel & wheel, const EncoderSnapshot & snapshot, double now_s)
{
  WheelOutput out;
  out.target_rpm = wheel.target_rpm;
  out.encoder = snapshot;
  out.count_delta = snapshot.count - wheel.last_count;
  out.valid_delta = snapshot.valid_transition_count - wheel.last_valid;
  out.invalid_delta = snapshot.invalid_transition_count - wheel.last_invalid;
  out.a_edge_delta = snapshot.a_edge_count - wheel.last_a_edges;
  out.b_edge_delta = snapshot.b_edge_count - wheel.last_b_edges;
  if (wheel.last_time_s == 0.0) wheel.last_time_s = now_s;
  const double dt = now_s - wheel.last_time_s;
  if (dt <= 0.0) {
    out.measured_rpm = wheel.last_measured_rpm;
    // Python's first update returns before applying the deferred startup
    // command when dt <= 0.  Keep the pending logical value internal until
    // the first real control interval.
    out.logical_pwm_command = wheel.pending_start ? 0.0 : wheel.logical_command;
    return out;
  }
  out.measured_rpm = static_cast<double>(out.count_delta) / encoder_cpr_ / dt * 60.0;
  if (wheel.command_direction == 0) {
    wheel.logical_command = 0.0;
  } else {
    if (wheel.pending_start) {
      // The Python implementation writes the bounded startup command first,
      // then continues through the normal PI calculation in this same real
      // control interval.  The final output of the interval is therefore the
      // PI result, not the temporary startup value.
      wheel.pending_start = false;
    }
    const double error = wheel.target_abs_rpm - std::abs(out.measured_rpm);
    wheel.integral = std::clamp(wheel.integral + error * dt, -30.0, 30.0);
    const double base = wheel.target_abs_rpm / kMaxRpmEstimate;
    wheel.logical_command = wheel.command_direction *
      std::clamp(base + kKp * error + kKi * wheel.integral, 0.0, 1.0);
  }
  wheel.last_count = snapshot.count;
  wheel.last_valid = snapshot.valid_transition_count;
  wheel.last_invalid = snapshot.invalid_transition_count;
  wheel.last_a_edges = snapshot.a_edge_count;
  wheel.last_b_edges = snapshot.b_edge_count;
  wheel.last_time_s = now_s;
  wheel.last_measured_rpm = out.measured_rpm;
  wheel.health_samples.push_back({now_s, static_cast<double>(out.count_delta),
    static_cast<double>(out.valid_delta), static_cast<double>(out.invalid_delta),
    static_cast<double>(out.a_edge_delta), static_cast<double>(out.b_edge_delta)});
  while (!wheel.health_samples.empty() &&
         wheel.health_samples.front()[0] < now_s - safety_.window_s) {
    wheel.health_samples.pop_front();
  }
  out.logical_pwm_command = wheel.logical_command;
  return out;
}

double DiffDriveControlCore::window_sum(const Wheel & wheel, int index) const
{
  double total = 0.0;
  for (const auto & sample : wheel.health_samples) total += sample[index];
  return total;
}

double DiffDriveControlCore::age_s(std::uint64_t timestamp_ns, double now_s) const
{
  if (timestamp_ns == 0) return INFINITY;
  const double event_s = static_cast<double>(timestamp_ns) / 1e9;
  return std::max(0.0, now_s - event_s);
}

void DiffDriveControlCore::evaluate_safety(const ControlOutput & output, double now_s)
{
  if (!safety_.enabled || fault_latched_) return;
  struct Candidate { int priority; std::string reason; std::string detail; bool no_pulse; };
  const auto check = [&](const char * side, Wheel & wheel,
                         const WheelOutput & state) {
    std::vector<Candidate> candidates;
    if (std::abs(wheel.target_rpm) < safety_.min_target_rpm ||
        std::abs(state.logical_pwm_command) < safety_.min_pwm ||
        now_s < wheel.grace_until_s) {
      wheel.underspeed_since_s = -1.0;
      wheel.implausible_cycles = 0;
      return candidates;
    }
    const double a = window_sum(wheel, 4);
    const double b = window_sum(wheel, 5);
    const double count_window = window_sum(wheel, 1);
    const double valid_window = window_sum(wheel, 2);
    const double max_count = std::max(a, b);
    const double min_count = std::min(a, b);
    if (max_count >= safety_.min_window_pulses &&
        min_count / max_count < safety_.channel_ratio_min) {
      const char * missing = a < b ? "A" : "B";
      std::ostringstream detail;
      detail << "window=" << safety_.window_s << "s A_edges=" << a
             << " B_edges=" << b << " ratio=" << min_count / max_count
             << " valid=" << valid_window << " target=" << wheel.target_rpm
             << "rpm pwm=" << state.logical_pwm_command;
      candidates.push_back({10, std::string(side) + "_ENCODER_" + missing + "_SIGNAL_LOSS",
        detail.str(), false});
    }
    const bool stopped = age_s(state.encoder.last_a_edge_time_ns, now_s) >= safety_.no_pulse_timeout_s &&
      age_s(state.encoder.last_b_edge_time_ns, now_s) >= safety_.no_pulse_timeout_s;
    if (stopped) {
      std::ostringstream detail;
      detail << "A_age=" << age_s(state.encoder.last_a_edge_time_ns, now_s)
             << "s B_age=" << age_s(state.encoder.last_b_edge_time_ns, now_s)
             << "s target=" << wheel.target_rpm << "rpm measured=" << state.measured_rpm;
      candidates.push_back({20, std::string(side) + "_WHEEL_FEEDBACK_LOSS_OR_STALL",
        detail.str(), true});
    }
    if (safety_.direction_check_enabled && valid_window >= safety_.direction_min_samples) {
      if (wheel.command_direction != 0 && count_window * wheel.command_direction < 0.0) {
        std::ostringstream detail;
        detail << "decoded_delta=" << count_window << " command_direction="
               << wheel.command_direction << " valid=" << valid_window
               << " target=" << wheel.target_rpm << "rpm";
        candidates.push_back({30, std::string(side) + "_ENCODER_DIRECTION_MISMATCH",
          detail.str(), false});
      }
    }
    if (std::abs(state.measured_rpm) > safety_.max_encoder_rpm) {
      ++wheel.implausible_cycles;
    } else {
      wheel.implausible_cycles = 0;
    }
    if (wheel.implausible_cycles >= safety_.implausible_rpm_cycles) {
      std::ostringstream detail;
      detail << "measured=" << state.measured_rpm << "rpm limit="
             << safety_.max_encoder_rpm << "rpm count_delta=" << state.count_delta
             << " invalid=" << state.invalid_delta;
      candidates.push_back({40, std::string(side) + "_ENCODER_IMPLAUSIBLE_JUMP",
        detail.str(), false});
    }
    const bool underspeed = std::abs(state.logical_pwm_command) >= safety_.underspeed_pwm &&
      std::abs(state.measured_rpm) < std::abs(wheel.target_rpm) * safety_.underspeed_ratio &&
      !stopped;
    if (underspeed) {
      if (wheel.underspeed_since_s < 0.0) wheel.underspeed_since_s = now_s;
      if (now_s - wheel.underspeed_since_s >= safety_.underspeed_timeout_s) {
        std::ostringstream detail;
        detail << "duration=" << now_s - wheel.underspeed_since_s << "s target="
               << wheel.target_rpm << "rpm measured=" << state.measured_rpm
               << "rpm pwm=" << state.logical_pwm_command;
        candidates.push_back({50, std::string(side) + "_WHEEL_SEVERE_UNDERSPEED",
          detail.str(), false});
      }
    } else {
      wheel.underspeed_since_s = -1.0;
    }
    return candidates;
  };
  auto left_candidates = check("LEFT", left_, output.left);
  auto right_candidates = check("RIGHT", right_, output.right);
  const auto find_no_pulse = [](const auto & candidates) -> const Candidate * {
    for (const auto & candidate : candidates) if (candidate.no_pulse) return &candidate;
    return nullptr;
  };
  const Candidate * left_no_pulse = find_no_pulse(left_candidates);
  const Candidate * right_no_pulse = find_no_pulse(right_candidates);
  if (left_no_pulse && right_no_pulse) {
    latch_fault("BOTH_WHEELS_FEEDBACK_LOSS_OR_STALL",
      "left[" + left_no_pulse->detail + "] right[" + right_no_pulse->detail + "]");
    return;
  }
  std::vector<Candidate> all;
  all.insert(all.end(), left_candidates.begin(), left_candidates.end());
  all.insert(all.end(), right_candidates.begin(), right_candidates.end());
  if (!all.empty()) {
    const auto selected = std::min_element(all.begin(), all.end(),
      [](const Candidate & a, const Candidate & b) { return a.priority < b.priority; });
    latch_fault(selected->reason, selected->detail);
  }
}

ControlOutput DiffDriveControlCore::step(
  double now_s, const EncoderSnapshot & left, const EncoderSnapshot & right)
{
  if (last_command_time_s_ != 0.0 && now_s - last_command_time_s_ > command_timeout_s_) {
    set_command(0.0, 0.0, now_s);
  }
  ControlOutput out;
  out.left = update_wheel(left_, left, now_s);
  out.right = update_wheel(right_, right, now_s);
  out.fault_latched = fault_latched_;
  out.fault_reason = fault_reason_;
  out.fault_detail = fault_detail_;
  evaluate_safety(out, now_s);
  if (fault_latched_) {
    out.left.logical_pwm_command = 0.0;
    out.right.logical_pwm_command = 0.0;
  }
  out.fault_latched = fault_latched_;
  out.fault_reason = fault_reason_;
  out.fault_detail = fault_detail_;

  const double dt = last_odom_time_s_ == 0.0 ? 0.0 : now_s - last_odom_time_s_;
  last_odom_time_s_ = now_s;
  if (dt > 0.0) {
    const double vl = rpm_to_mps(out.left.measured_rpm, wheel_radius_m_);
    const double vr = rpm_to_mps(out.right.measured_rpm, wheel_radius_m_);
    out.v = (vr + vl) / 2.0;
    out.omega = (vr - vl) / wheel_separation_odom_m_;
    const double dtheta = out.omega * dt;
    const double mid = theta_ + dtheta / 2.0;
    x_ += out.v * std::cos(mid) * dt;
    y_ += out.v * std::sin(mid) * dt;
    theta_ = std::atan2(std::sin(theta_ + dtheta), std::cos(theta_ + dtheta));
  }
  out.x = x_; out.y = y_; out.theta = theta_;
  return out;
}

void DiffDriveControlCore::latch_fault(const std::string & reason, const std::string & detail)
{
  if (fault_latched_) return;
  fault_latched_ = true;
  fault_reason_ = reason;
  fault_detail_ = detail;
  left_.target_rpm = right_.target_rpm = 0.0;
  left_.command_direction = right_.command_direction = 0;
  left_.logical_command = right_.logical_command = 0.0;
}

void DiffDriveControlCore::clear_fault_for_test()
{
  fault_latched_ = false;
  fault_reason_.clear();
  fault_detail_.clear();
}

}  // namespace my_epuck_project_cpp
