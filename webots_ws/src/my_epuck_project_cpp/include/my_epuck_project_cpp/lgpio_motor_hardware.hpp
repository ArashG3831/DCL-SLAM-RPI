#pragma once

#include "my_epuck_project_cpp/lgpio_compat.hpp"
#include "my_epuck_project_cpp/quadrature_decoder.hpp"

#include <array>

namespace my_epuck_project_cpp {

class LgpioMotorHardware {
public:
  LgpioMotorHardware(QuadratureDecoder & left, QuadratureDecoder & right);
  ~LgpioMotorHardware();
  LgpioMotorHardware(const LgpioMotorHardware &) = delete;
  LgpioMotorHardware & operator=(const LgpioMotorHardware &) = delete;

  void set_motor(double logical_command, int motor_sign, int enable_gpio,
                 int in1_gpio, int in2_gpio);
  void stop_all();
  bool valid() const { return handle_ >= 0; }

private:
  struct Binding { QuadratureDecoder * decoder; char channel; };
  static void alert_callback(int count, lgGpioAlertPtr alerts, void * userdata);
  void claim_encoder(int gpio, Binding & binding);
  void claim_output(int gpio);

  int handle_{-1};
  std::array<int, 8> claimed_lines_{};
  std::size_t claimed_count_{0};
  Binding left_a_{}, left_b_{}, right_a_{}, right_b_{};
};

}  // namespace my_epuck_project_cpp
