#include "my_epuck_project_cpp/d500_parser.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <cerrno>
#include <fcntl.h>
#include <filesystem>
#include <mutex>
#include <optional>
#include <poll.h>
#include <string>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <vector>

namespace my_epuck_project_cpp {

constexpr double kPi = 3.14159265358979323846;

double system_wall_seconds()
{
  return std::chrono::duration<double>(
    std::chrono::system_clock::now().time_since_epoch()).count();
}

class D500Ros2ScanCpp final : public rclcpp::Node {
public:
  D500Ros2ScanCpp()
  : Node("d500_ros2_scan")
  {
    port_ = declare_parameter<std::string>("port", find_default_port());
    baud_ = declare_parameter<int>("baud", 230400);
    topic_ = declare_parameter<std::string>("topic", "/scan");
    frame_id_ = declare_parameter<std::string>("frame_id", "laser");
    bins_ = declare_parameter<int>("bins", 720);
    min_mm_ = declare_parameter<int>("min_mm", 30);
    max_mm_ = declare_parameter<int>("max_mm", 12000);
    min_intensity_ = declare_parameter<int>("min_intensity", 0);
    invert_ = declare_parameter<bool>("invert", false);
    angle_offset_deg_ = declare_parameter<double>("angle_offset_deg", 0.0);
    scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(topic_, 10);
    parser_ = std::make_unique<D500StreamParser>(bins_, min_mm_, max_mm_, min_intensity_, invert_, angle_offset_deg_);
    worker_ = std::thread(&D500Ros2ScanCpp::reader_loop, this);
    publish_timer_ = create_wall_timer(std::chrono::milliseconds(5), [this]() { publish_latest(); });
    diagnostic_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() { publish_diagnostics(); });
    RCLCPP_INFO(get_logger(), "d500_ros2_scan_cpp opt-in backend: %s @ %d, bins=%d", port_.c_str(), baud_, bins_);
  }

  ~D500Ros2ScanCpp() override
  {
    stopping_.store(true);
    if (fd_ >= 0) close(fd_);
    if (worker_.joinable()) worker_.join();
  }

private:
  static std::string find_default_port()
  {
    const std::filesystem::path by_id("/dev/serial/by-id");
    try {
      if (std::filesystem::exists(by_id)) {
        std::vector<std::string> candidates;
        for (const auto & entry : std::filesystem::directory_iterator(by_id)) {
          candidates.push_back(entry.path().string());
        }
        if (!candidates.empty()) {
          std::sort(candidates.begin(), candidates.end());
          return candidates.front();
        }
      }
    } catch (const std::filesystem::filesystem_error &) {
      // Preserve the Python fallback if /dev/serial is unavailable.
    }
    return "/dev/ttyUSB0";
  }

  static speed_t baud_constant(int baud)
  {
    return baud == 230400 ? B230400 : baud == 115200 ? B115200 : B9600;
  }

  void reader_loop()
  {
    fd_ = open(port_.c_str(), O_RDONLY | O_NOCTTY | O_CLOEXEC);
    if (fd_ < 0) {
      ++serial_error_count_;
      RCLCPP_ERROR(get_logger(), "cannot open D500 port %s", port_.c_str());
      return;
    }
    termios tty{};
    if (tcgetattr(fd_, &tty) != 0) {
      ++serial_error_count_;
      RCLCPP_ERROR(get_logger(), "tcgetattr failed");
      return;
    }
    cfmakeraw(&tty); cfsetispeed(&tty, baud_constant(baud_)); cfsetospeed(&tty, baud_constant(baud_));
    tty.c_cflag |= (CLOCAL | CREAD); tty.c_cflag &= ~CSTOPB; tty.c_cflag &= ~CRTSCTS;
    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
      ++serial_error_count_;
      RCLCPP_ERROR(get_logger(), "tcsetattr failed");
      return;
    }
    tcflush(fd_, TCIFLUSH);
    std::array<std::uint8_t, 4096> bytes{};
    while (!stopping_.load()) {
      pollfd pfd{fd_, POLLIN, 0};
      const int ready = poll(&pfd, 1, 50);
      if (ready <= 0) continue;
      const auto n = read(fd_, bytes.data(), bytes.size());
      if (n > 0) {
        std::lock_guard<std::mutex> lock(parser_mutex_);
        parser_->feed(bytes.data(), static_cast<std::size_t>(n), system_wall_seconds());
        CompletedScan scan;
        while (parser_->take_completed_scan(scan)) {
          std::lock_guard<std::mutex> handoff(handoff_mutex_);
          if (latest_scan_) ++handoff_drops_;
          latest_scan_ = std::move(scan);
        }
      } else if (n < 0 && !stopping_.load() && errno != EINTR && errno != EAGAIN) {
        ++serial_error_count_;
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                              "D500 serial read failed: %s", std::strerror(errno));
      }
    }
  }

  static builtin_interfaces::msg::Time ros_time_from_wall(double seconds)
  {
    builtin_interfaces::msg::Time t;
    const auto sec = static_cast<std::int64_t>(seconds);
    t.sec = static_cast<std::int32_t>(sec);
    t.nanosec = static_cast<std::uint32_t>((seconds - static_cast<double>(sec)) * 1e9);
    return t;
  }

  void publish_latest()
  {
    std::optional<CompletedScan> scan;
    { std::lock_guard<std::mutex> lock(handoff_mutex_); if (latest_scan_) { scan = std::move(latest_scan_); latest_scan_.reset(); } }
    if (!scan) return;
    sensor_msgs::msg::LaserScan raw;
    raw.header.stamp = ros_time_from_wall(scan->acquisition_midpoint_wall);
    raw.header.frame_id = frame_id_;
    raw.angle_min = 0.0F; raw.angle_max = static_cast<float>(2.0 * kPi);
    raw.angle_increment = static_cast<float>(2.0 * kPi / static_cast<double>(bins_));
    raw.scan_time = static_cast<float>(scan->scan_time > 0.0 ? scan->scan_time : 0.1);
    raw.time_increment = raw.scan_time / static_cast<float>(bins_);
    raw.range_min = min_mm_ / 1000.0F; raw.range_max = max_mm_ / 1000.0F;
    raw.ranges = scan->ranges; raw.intensities = scan->intensities;
    sensor_msgs::msg::LaserScan out = mirror(raw);
    scan_pub_->publish(out);
    const double now = system_wall_seconds();
    if (last_publish_wall_ > 0.0) {
      const double gap = now - last_publish_wall_;
      max_publish_gap_ = std::max(max_publish_gap_, gap);
    }
    last_publish_wall_ = now;
    last_scan_age_ = std::max(0.0, now - scan->acquisition_midpoint_wall);
    ++published_scan_count_;
  }

  void publish_diagnostics()
  {
    std::uint64_t packets, crc, parser_errors, completed, handoff_drops;
    double last_scan_time, max_scan_time, rpm;
    {
      std::lock_guard<std::mutex> lock(parser_mutex_);
      packets = parser_->packet_count();
      crc = parser_->bad_crc_count();
      parser_errors = parser_->parser_error_count();
      completed = parser_->completed_scan_count();
      last_scan_time = parser_->last_scan_time();
      max_scan_time = parser_->max_scan_time();
      rpm = parser_->last_rpm();
    }
    {
      std::lock_guard<std::mutex> lock(handoff_mutex_);
      handoff_drops = handoff_drops_;
    }
    RCLCPP_INFO(get_logger(),
      "D500_CPP scans=%zu published=%zu packets=%zu acq_last=%.1fms acq_max=%.1fms "
      "pub_gap_max=%.1fms scan_age=%.1fms rpm=%.1f crc=%zu parser=%zu "
      "handoff_drops=%zu serial_errors=%zu",
      completed, published_scan_count_.load(), packets, last_scan_time * 1000.0,
      max_scan_time * 1000.0, max_publish_gap_ * 1000.0, last_scan_age_ * 1000.0,
      rpm, crc, parser_errors, handoff_drops, serial_error_count_.load());
  }

  static sensor_msgs::msg::LaserScan mirror(const sensor_msgs::msg::LaserScan & msg)
  {
    const std::size_t n = msg.ranges.size(); if (n == 0) return msg;
    sensor_msgs::msg::LaserScan out; out.header = msg.header;
    out.angle_min = -static_cast<float>(kPi); out.angle_increment = static_cast<float>(2.0 * kPi / n);
    out.angle_max = out.angle_min + out.angle_increment * static_cast<float>(n - 1);
    out.time_increment = msg.time_increment; out.scan_time = msg.scan_time;
    out.range_min = msg.range_min; out.range_max = msg.range_max;
    out.ranges.assign(n, INFINITY);
    if (msg.intensities.size() == n) out.intensities.assign(n, 0.0F);
    for (std::size_t i = 0; i < n; ++i) {
      const double theta_out = out.angle_min + static_cast<double>(i) * out.angle_increment;
      const double theta_raw = std::fmod(-theta_out + 2.0 * kPi, 2.0 * kPi);
      const std::size_t j = static_cast<std::size_t>(std::llround((theta_raw - msg.angle_min) / msg.angle_increment)) % n;
      out.ranges[i] = msg.ranges[j]; if (!out.intensities.empty()) out.intensities[i] = msg.intensities[j];
    }
    return out;
  }

  std::string port_, topic_, frame_id_;
  int baud_{230400}, bins_{720}, min_mm_{30}, max_mm_{12000}, min_intensity_{0};
  bool invert_{false}; double angle_offset_deg_{0.0};
  int fd_{-1}; std::atomic<bool> stopping_{false};
  std::atomic<std::size_t> published_scan_count_{0};
  std::atomic<std::size_t> serial_error_count_{0};
  double last_publish_wall_{0.0};
  double max_publish_gap_{0.0};
  double last_scan_age_{0.0};
  std::unique_ptr<D500StreamParser> parser_; std::mutex parser_mutex_;
  std::thread worker_; std::mutex handoff_mutex_; std::optional<CompletedScan> latest_scan_;
  std::uint64_t handoff_drops_{0};
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr diagnostic_timer_;
};

}  // namespace my_epuck_project_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<my_epuck_project_cpp::D500Ros2ScanCpp>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
