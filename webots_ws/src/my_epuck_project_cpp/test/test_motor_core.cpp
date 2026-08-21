#include "my_epuck_project_cpp/diffdrive_control_core.hpp"
#include "my_epuck_project_cpp/quadrature_decoder.hpp"

#include <gtest/gtest.h>

using namespace my_epuck_project_cpp;

TEST(Quadrature, ForwardReverseAndInvalidParity)
{
  QuadratureDecoder d(0, 1);
  for (auto s : {1U, 3U, 2U, 0U}) d.process_state(s);
  EXPECT_EQ(d.snapshot().count, 4);
  d.process_state(3U);
  EXPECT_EQ(d.snapshot().count, 4);
  EXPECT_EQ(d.snapshot().invalid_transition_count, 1U);
  // The invalid 0->3 jump leaves the decoder at state 3.  Reset only the
  // decoder phase (not the counters) before beginning an independent reverse
  // sequence, matching how a new fixture segment is initialized.
  d.initialize_state(0U);
  for (auto s : {2U, 3U, 1U, 0U}) d.process_state(s);
  EXPECT_EQ(d.snapshot().count, 0);
}

TEST(Quadrature, EdgeCountersIncludeDuplicatesButNotAsMotion)
{
  QuadratureDecoder d(0, 1);
  d.process_edge('B', 0);  // duplicate notification
  d.process_edge('B', 1);
  d.process_edge('A', 1);
  d.process_edge('B', 0);
  d.process_edge('A', 0);
  const auto s = d.snapshot();
  EXPECT_EQ(s.count, 4);
  EXPECT_EQ(s.a_edge_count, 2U);
  EXPECT_EQ(s.b_edge_count, 3U);
  EXPECT_EQ(s.invalid_transition_count, 0U);
}

TEST(Conversion, CprAndDeadbandAreFrozen)
{
  EXPECT_NEAR(rpm_to_mps(50.0), 0.1832595715, 1e-9);
  EXPECT_DOUBLE_EQ(apply_motor_deadband_rpm(0.0), 0.0);
  EXPECT_DOUBLE_EQ(apply_motor_deadband_rpm(1.0), 12.0);
  EXPECT_DOUBLE_EQ(apply_motor_deadband_rpm(-1.0), -12.0);
}

TEST(Kinematics, CommandTargetsUseCalibratedSeparation)
{
  DiffDriveControlCore core;
  core.set_command(0.0, 1.0, 1.0);
  auto out = core.step(1.0, {}, {});
  const double expected = mps_to_rpm(kWheelSeparationCmdM / 2.0);
  EXPECT_NEAR(out.left.target_rpm, -expected, 1e-9);
  EXPECT_NEAR(out.right.target_rpm, expected, 1e-9);
}

TEST(Odometry, StraightAndSpin)
{
  DiffDriveControlCore straight;
  straight.set_command(0.0, 0.0, 1.0);
  EncoderSnapshot l{}, r{};
  auto a = straight.step(1.0, l, r);
  l.count = 4606; r.count = 4606;
  auto b = straight.step(2.0, l, r);
  EXPECT_NEAR(b.theta, 0.0, 1e-12);
  EXPECT_GT(b.x, 0.0);
  (void)a;
}

TEST(Safety, StartupGraceThenDualNoPulseLatchesAndZerosOutput)
{
  DiffDriveControlCore core;
  core.set_command(0.1832595715, 0.0, 1.0);

  EncoderSnapshot quiet;
  quiet.last_a_edge_time_ns = 1000000000ULL;
  quiet.last_b_edge_time_ns = 1000000000ULL;

  // The initial 0.60 s grace is still active.
  auto during_grace = core.step(1.50, quiet, quiet);
  EXPECT_FALSE(during_grace.fault_latched);

  // Both channels have been silent beyond 0.35 s after grace.
  auto fault = core.step(1.70, quiet, quiet);
  EXPECT_TRUE(fault.fault_latched);
  EXPECT_EQ(fault.fault_reason, "BOTH_WHEELS_FEEDBACK_LOSS_OR_STALL");
  EXPECT_DOUBLE_EQ(fault.left.logical_pwm_command, 0.0);
  EXPECT_DOUBLE_EQ(fault.right.logical_pwm_command, 0.0);
}

TEST(Safety, FaultRemainsLatchedAgainstLaterCommand)
{
  DiffDriveControlCore core;
  core.latch_fault("TEST_FAULT", "synthetic");
  core.set_command(0.1832595715, 0.0, 2.0);
  const auto out = core.step(2.0, {}, {});
  EXPECT_TRUE(out.fault_latched);
  EXPECT_EQ(out.fault_reason, "TEST_FAULT");
  EXPECT_DOUBLE_EQ(out.left.logical_pwm_command, 0.0);
  EXPECT_DOUBLE_EQ(out.right.logical_pwm_command, 0.0);
}
