#include "my_epuck_project_cpp/d500_parser.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace my_epuck_project_cpp {

namespace {
std::array<std::uint8_t, 256> make_crc_table()
{
  std::array<std::uint8_t, 256> table{};
  for (int i = 0; i < 256; ++i) {
    std::uint8_t c = static_cast<std::uint8_t>(i);
    for (int bit = 0; bit < 8; ++bit) {
      c = (c & 0x80U) != 0U ? static_cast<std::uint8_t>((c << 1U) ^ 0x4DU)
                            : static_cast<std::uint8_t>(c << 1U);
    }
    table[static_cast<std::size_t>(i)] = c;
  }
  return table;
}
const auto kCrcTable = make_crc_table();

std::uint16_t read_u16_le(const std::uint8_t * p)
{
  return static_cast<std::uint16_t>(p[0] | (static_cast<std::uint16_t>(p[1]) << 8U));
}
}  // namespace

std::uint8_t d500_crc8(const std::uint8_t * data, std::size_t length)
{
  std::uint8_t c = 0;
  for (std::size_t i = 0; i < length; ++i) c = kCrcTable[(c ^ data[i]) & 0xffU];
  return c;
}

bool parse_d500_packet(
  const std::array<std::uint8_t, kD500PacketLength> & packet, D500Packet & result)
{
  if (packet[0] != kD500Header0 || packet[1] != kD500VerLen ||
      d500_crc8(packet.data(), kD500PacketLength - 1) != packet.back()) return false;
  const std::uint16_t speed_dps = read_u16_le(packet.data() + 2);
  const double start_deg = (read_u16_le(packet.data() + 4) % 36000U) / 100.0;
  for (std::size_t i = 0; i < kD500Points; ++i) {
    const auto * p = packet.data() + 6 + i * 3;
    result.points[i] = {read_u16_le(p), p[2]};
  }
  const double end_deg = (read_u16_le(packet.data() + 42) % 36000U) / 100.0;
  const double diff = std::fmod(end_deg - start_deg + 360.0, 360.0);
  const double step = diff / static_cast<double>(kD500Points - 1);
  result.rpm = speed_dps / 6.0;
  for (std::size_t i = 0; i < kD500Points; ++i) {
    result.angles_deg[i] = std::fmod(start_deg + static_cast<double>(i) * step, 360.0);
  }
  result.sensor_timestamp = read_u16_le(packet.data() + 44);
  return true;
}

bool is_d500_wrap(double previous_deg, double current_deg, bool have_previous)
{
  return have_previous && previous_deg - current_deg > 180.0;
}

D500StreamParser::D500StreamParser(int bins, int min_mm, int max_mm, int min_intensity,
                                   bool invert, double angle_offset_deg)
: bins_(bins), min_mm_(min_mm), max_mm_(max_mm), min_intensity_(min_intensity),
  invert_(invert), angle_offset_deg_(angle_offset_deg), current_ranges_(bins, INFINITY),
  current_intensities_(bins, 0.0F)
{
  buffer_.reserve(kD500PacketLength * 8);
}

int D500StreamParser::angle_to_bin(double angle_deg) const
{
  double a = std::fmod(angle_deg + angle_offset_deg_, 360.0);
  if (a < 0.0) a += 360.0;
  if (invert_) a = std::fmod(360.0 - a, 360.0);
  return static_cast<int>((a / 360.0) * bins_) % bins_;
}

void D500StreamParser::reset_scan()
{
  std::fill(current_ranges_.begin(), current_ranges_.end(), INFINITY);
  std::fill(current_intensities_.begin(), current_intensities_.end(), 0.0F);
}

void D500StreamParser::update_point(double angle_deg, std::uint16_t distance_mm,
                                    std::uint8_t intensity)
{
  if (distance_mm == 0 || distance_mm < min_mm_ || distance_mm > max_mm_ ||
      intensity < min_intensity_) return;
  const int index = angle_to_bin(angle_deg);
  const float range_m = static_cast<float>(distance_mm) / 1000.0F;
  if (range_m < current_ranges_[static_cast<std::size_t>(index)]) {
    current_ranges_[static_cast<std::size_t>(index)] = range_m;
    current_intensities_[static_cast<std::size_t>(index)] = static_cast<float>(intensity);
  }
}

void D500StreamParser::process_packet(
  const std::array<std::uint8_t, kD500PacketLength> & bytes, double wall_time_s)
{
  D500Packet packet;
  if (!parse_d500_packet(bytes, packet)) { ++bad_crc_count_; return; }
  ++packet_count_;
  last_rpm_ = packet.rpm;
  for (std::size_t i = 0; i < kD500Points; ++i) {
    const double angle = packet.angles_deg[i];
    const auto point = packet.points[i];
    if (!started_) {
      if (point.distance_mm > 0) {
        started_ = true;
        have_last_angle_ = true;
        last_angle_deg_ = angle;
        revolution_start_wall_ = wall_time_s;
      }
      update_point(angle, point.distance_mm, point.intensity);
      continue;
    }
    if (is_d500_wrap(last_angle_deg_, angle, have_last_angle_)) {
      const double end = wall_time_s;
      const double duration = std::max(0.0, end - revolution_start_wall_);
      completed_.ranges = current_ranges_;
      completed_.intensities = current_intensities_;
      completed_.acquisition_start_wall = revolution_start_wall_;
      completed_.acquisition_end_wall = end;
      completed_.acquisition_midpoint_wall = revolution_start_wall_ + duration / 2.0;
      completed_.scan_time = duration;
      completed_.rpm = packet.rpm;
      completed_.valid_bins = static_cast<std::size_t>(std::count_if(
        current_ranges_.begin(), current_ranges_.end(), [](float x) { return std::isfinite(x); }));
      completed_ready_ = true;
      ++completed_scan_count_;
      last_scan_time_ = duration;
      max_scan_time_ = std::max(max_scan_time_, duration);
      last_rpm_ = packet.rpm;
      reset_scan();
      revolution_start_wall_ = end;
    }
    last_angle_deg_ = angle;
    update_point(angle, point.distance_mm, point.intensity);
  }
}

void D500StreamParser::feed(const std::uint8_t * bytes, std::size_t length, double wall_time_s)
{
  buffer_.insert(buffer_.end(), bytes, bytes + length);
  static constexpr std::array<std::uint8_t, 2> kHeader{kD500Header0, kD500VerLen};
  for (;;) {
    auto it = std::search(buffer_.begin(), buffer_.end(), kHeader.begin(), kHeader.end());
    if (it == buffer_.end()) {
      if (buffer_.size() > 1) buffer_.erase(buffer_.begin(), buffer_.end() - 1);
      return;
    }
    const std::size_t offset = static_cast<std::size_t>(it - buffer_.begin());
    if (buffer_.size() - offset < kD500PacketLength) {
      if (offset > 0) buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(offset));
      return;
    }
    std::array<std::uint8_t, kD500PacketLength> packet{};
    std::copy_n(buffer_.begin() + static_cast<std::ptrdiff_t>(offset),
                kD500PacketLength, packet.begin());
    buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(offset + kD500PacketLength));
    process_packet(packet, wall_time_s);
  }
}

bool D500StreamParser::take_completed_scan(CompletedScan & scan)
{
  if (!completed_ready_) return false;
  scan = std::move(completed_);
  completed_ready_ = false;
  return true;
}

}  // namespace my_epuck_project_cpp
