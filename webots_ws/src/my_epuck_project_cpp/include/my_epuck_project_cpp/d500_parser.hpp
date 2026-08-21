#pragma once

#include <array>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace my_epuck_project_cpp {

constexpr std::uint8_t kD500Header0 = 0x54;
constexpr std::uint8_t kD500VerLen = 0x2C;
constexpr std::size_t kD500Points = 12;
constexpr std::size_t kD500PacketLength = 47;

struct D500Point {
  std::uint16_t distance_mm{0};
  std::uint8_t intensity{0};
};

struct D500Packet {
  double rpm{0.0};
  std::array<double, kD500Points> angles_deg{};
  std::array<D500Point, kD500Points> points{};
  std::uint16_t sensor_timestamp{0};
};

struct CompletedScan {
  std::vector<float> ranges;
  std::vector<float> intensities;
  double acquisition_start_wall{0.0};
  double acquisition_end_wall{0.0};
  double acquisition_midpoint_wall{0.0};
  double scan_time{0.0};
  double rpm{0.0};
  std::size_t valid_bins{0};
};

std::uint8_t d500_crc8(const std::uint8_t * data, std::size_t length);
bool parse_d500_packet(const std::array<std::uint8_t, kD500PacketLength> & packet,
                       D500Packet & result);
bool is_d500_wrap(double previous_deg, double current_deg, bool have_previous);

class D500StreamParser {
public:
  D500StreamParser(int bins = 720, int min_mm = 30, int max_mm = 12000,
                   int min_intensity = 0, bool invert = false,
                   double angle_offset_deg = 0.0);

  void feed(const std::uint8_t * bytes, std::size_t length, double wall_time_s);
  void feed(const std::vector<std::uint8_t> & bytes, double wall_time_s)
  { feed(bytes.data(), bytes.size(), wall_time_s); }
  bool take_completed_scan(CompletedScan & scan);
  std::uint64_t packet_count() const { return packet_count_; }
  std::uint64_t bad_crc_count() const { return bad_crc_count_; }
  std::uint64_t parser_error_count() const { return parser_error_count_; }
  std::uint64_t completed_scan_count() const { return completed_scan_count_; }
  double last_scan_time() const { return last_scan_time_; }
  double max_scan_time() const { return max_scan_time_; }
  double last_rpm() const { return last_rpm_; }

private:
  int angle_to_bin(double angle_deg) const;
  void reset_scan();
  void update_point(double angle_deg, std::uint16_t distance_mm, std::uint8_t intensity);
  void process_packet(const std::array<std::uint8_t, kD500PacketLength> & packet,
                      double wall_time_s);

  const int bins_;
  const int min_mm_;
  const int max_mm_;
  const int min_intensity_;
  const bool invert_;
  const double angle_offset_deg_;
  std::vector<std::uint8_t> buffer_;
  std::vector<float> current_ranges_;
  std::vector<float> current_intensities_;
  bool started_{false};
  bool have_last_angle_{false};
  double last_angle_deg_{0.0};
  double revolution_start_wall_{0.0};
  CompletedScan completed_;
  bool completed_ready_{false};
  std::uint64_t packet_count_{0};
  std::uint64_t bad_crc_count_{0};
  std::uint64_t parser_error_count_{0};
  std::uint64_t completed_scan_count_{0};
  double last_scan_time_{0.0};
  double max_scan_time_{0.0};
  double last_rpm_{0.0};
};

}  // namespace my_epuck_project_cpp
