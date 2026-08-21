#include "my_epuck_project_cpp/lgpio_motor_hardware.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace my_epuck_project_cpp {

namespace {
constexpr int kRightPwm = 18, kRightIn1 = 23, kRightIn2 = 24;
constexpr int kRightA = 17, kRightB = 27;
constexpr int kLeftPwm = 13, kLeftIn1 = 5, kLeftIn2 = 6;
constexpr int kLeftA = 22, kLeftB = 26;
constexpr int kPwmHz = 1000;
constexpr int kMotorM1Sign = 1;
constexpr int kMotorM2Sign = -1;
}

LgpioMotorHardware::LgpioMotorHardware(QuadratureDecoder & left, QuadratureDecoder & right)
{
  left_a_ = {&left, 'A'}; left_b_ = {&left, 'B'};
  right_a_ = {&right, 'A'}; right_b_ = {&right, 'B'};
  handle_ = lgGpiochipOpen(0);
  if (handle_ < 0) throw std::runtime_error("lgGpiochipOpen(0) failed");
  try {
    claim_encoder(kLeftA, left_a_); claim_encoder(kLeftB, left_b_);
    claim_encoder(kRightA, right_a_); claim_encoder(kRightB, right_b_);
    const int left_a_level = lgGpioRead(handle_, kLeftA);
    const int left_b_level = lgGpioRead(handle_, kLeftB);
    const int right_a_level = lgGpioRead(handle_, kRightA);
    const int right_b_level = lgGpioRead(handle_, kRightB);
    if (left_a_level < 0 || left_b_level < 0 ||
        right_a_level < 0 || right_b_level < 0) {
      throw std::runtime_error("failed to read initial encoder levels");
    }
    // Match gpiozero's startup behavior: establish the four-bit phase before
    // counting events.  The initial electrical state is not wheel motion.
    left.initialize_state(static_cast<std::uint8_t>((left_a_level << 1) | left_b_level));
    right.initialize_state(static_cast<std::uint8_t>((right_a_level << 1) | right_b_level));
    claim_output(kLeftPwm); claim_output(kLeftIn1); claim_output(kLeftIn2);
    claim_output(kRightPwm); claim_output(kRightIn1); claim_output(kRightIn2);
    stop_all();
  } catch (...) {
    stop_all();
    for (std::size_t i = 0; i < claimed_count_; ++i) lgGpioFree(handle_, claimed_lines_[i]);
    lgGpiochipClose(handle_); handle_ = -1; throw;
  }
}

void LgpioMotorHardware::claim_output(int gpio)
{
  if (lgGpioClaimOutput(handle_, 0, gpio, 0) < 0) throw std::runtime_error("failed to claim GPIO output");
  claimed_lines_[claimed_count_++] = gpio;
}

void LgpioMotorHardware::claim_encoder(int gpio, Binding & binding)
{
  if (lgGpioClaimInput(handle_, LG_SET_PULL_UP, gpio) < 0) throw std::runtime_error("failed to claim encoder input");
  // Register the line before installing alerts so constructor failure cleanup
  // also releases a GPIO whose alert registration failed.
  claimed_lines_[claimed_count_++] = gpio;
  if (lgGpioSetAlertsFunc(handle_, gpio, &LgpioMotorHardware::alert_callback, &binding) < 0 ||
      lgGpioClaimAlert(handle_, 0, LG_BOTH_EDGES, gpio, -1) < 0) {
    throw std::runtime_error("failed to claim encoder alert");
  }
}

void LgpioMotorHardware::alert_callback(int count, lgGpioAlertPtr alerts, void * userdata)
{
  auto * binding = static_cast<Binding *>(userdata);
  for (int i = 0; i < count; ++i) {
    if (alerts[i].report.level <= 1) {
      binding->decoder->process_edge(binding->channel, alerts[i].report.level,
                                      alerts[i].report.timestamp);
    }
  }
}

void LgpioMotorHardware::set_motor(double logical_command, int motor_sign,
                                   int enable_gpio, int in1_gpio, int in2_gpio)
{
  const double command = std::clamp(logical_command, -1.0, 1.0) * motor_sign;
  if (command > 0.0) {
    lgGpioWrite(handle_, in1_gpio, 1); lgGpioWrite(handle_, in2_gpio, 0);
    lgTxPwm(handle_, enable_gpio, kPwmHz, static_cast<float>(std::abs(command) * 100.0), 0, 0);
  } else if (command < 0.0) {
    lgGpioWrite(handle_, in1_gpio, 0); lgGpioWrite(handle_, in2_gpio, 1);
    lgTxPwm(handle_, enable_gpio, kPwmHz, static_cast<float>(std::abs(command) * 100.0), 0, 0);
  } else {
    lgTxPwm(handle_, enable_gpio, 0.0F, 0.0F, 0, 0);
    lgGpioWrite(handle_, in1_gpio, 0); lgGpioWrite(handle_, in2_gpio, 0);
  }
}

void LgpioMotorHardware::stop_all()
{
  if (handle_ < 0) return;
  set_motor(0.0, kMotorM2Sign, kLeftPwm, kLeftIn1, kLeftIn2);
  set_motor(0.0, kMotorM1Sign, kRightPwm, kRightIn1, kRightIn2);
}

LgpioMotorHardware::~LgpioMotorHardware()
{
  if (handle_ < 0) return;
  stop_all();
  for (std::size_t i = 0; i < claimed_count_; ++i) lgGpioFree(handle_, claimed_lines_[i]);
  lgGpiochipClose(handle_);
}

}  // namespace my_epuck_project_cpp
