#include "my_epuck_project_cpp/diffdrive_control_core.hpp"

#include <iomanip>
#include <iostream>
#include <vector>

using namespace my_epuck_project_cpp;

int main()
{
  DiffDriveControlCore core;
  core.set_command(0.1832595715, 0.0, 1.0);
  std::int64_t count = 0;
  std::cout << "{\"steps\":[";
  const std::vector<double> times{1.0, 1.05, 1.10, 1.15, 1.20};
  for (std::size_t i = 0; i < times.size(); ++i) {
    if (i > 0) count += 192;
    EncoderSnapshot left, right;
    left.count = count; right.count = count;
    left.valid_transition_count = static_cast<std::uint64_t>(count);
    right.valid_transition_count = static_cast<std::uint64_t>(count);
    left.a_edge_count = right.a_edge_count = static_cast<std::uint64_t>(count / 2);
    left.b_edge_count = right.b_edge_count = static_cast<std::uint64_t>(count / 2);
    left.last_a_edge_time_ns = left.last_b_edge_time_ns = 1000000000ULL;
    right.last_a_edge_time_ns = right.last_b_edge_time_ns = 1000000000ULL;
    const auto out = core.step(times[i], left, right);
    if (i > 0) std::cout << ',';
    std::cout << std::setprecision(17)
              << "{\"t\":" << times[i]
              << ",\"left_target\":" << out.left.target_rpm
              << ",\"right_target\":" << out.right.target_rpm
              << ",\"left_rpm\":" << out.left.measured_rpm
              << ",\"right_rpm\":" << out.right.measured_rpm
              << ",\"left_count\":" << out.left.encoder.count
              << ",\"right_count\":" << out.right.encoder.count
              << ",\"left_valid\":" << out.left.encoder.valid_transition_count
              << ",\"right_valid\":" << out.right.encoder.valid_transition_count
              << ",\"left_invalid\":" << out.left.encoder.invalid_transition_count
              << ",\"right_invalid\":" << out.right.encoder.invalid_transition_count
              << ",\"left_command\":" << out.left.logical_pwm_command
              << ",\"right_command\":" << out.right.logical_pwm_command
              << ",\"x\":" << out.x << ",\"y\":" << out.y
              << ",\"theta\":" << out.theta << '}';
  }
  std::cout << "]}\n";
  return 0;
}
