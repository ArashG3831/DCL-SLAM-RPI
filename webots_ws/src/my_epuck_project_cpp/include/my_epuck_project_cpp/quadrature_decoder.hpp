#pragma once

#include <atomic>
#include <cstdint>

namespace my_epuck_project_cpp {

struct EncoderSnapshot {
  std::int64_t count{0};
  std::uint64_t valid_transition_count{0};
  std::uint64_t invalid_transition_count{0};
  std::uint64_t a_edge_count{0};
  std::uint64_t b_edge_count{0};
  std::int8_t last_delta{0};
  std::int8_t decoded_direction{0};
  std::uint64_t last_transition_time_ns{0};
  std::uint64_t last_a_edge_time_ns{0};
  std::uint64_t last_b_edge_time_ns{0};
  std::uint8_t a_level{0};
  std::uint8_t b_level{0};
  std::uint8_t current_state{0};
};

class QuadratureDecoder {
public:
  explicit QuadratureDecoder(std::uint8_t initial_state = 0, int encoder_sign = 1);

  int process_state(std::uint8_t new_state, std::uint64_t timestamp_ns = 0);
  int process_edge(char channel, std::uint8_t level, std::uint64_t timestamp_ns = 0);
  void initialize_state(std::uint8_t state);
  EncoderSnapshot snapshot() const;
  void reset_counts();

private:
  int transition_delta(std::uint8_t previous, std::uint8_t current) const;

  const int encoder_sign_;
  std::atomic<std::uint8_t> state_;
  std::atomic<std::int64_t> count_{0};
  std::atomic<std::uint64_t> valid_{0};
  std::atomic<std::uint64_t> invalid_{0};
  std::atomic<std::uint64_t> a_edges_{0};
  std::atomic<std::uint64_t> b_edges_{0};
  std::atomic<std::int8_t> last_delta_{0};
  std::atomic<std::int8_t> direction_{0};
  std::atomic<std::uint64_t> last_transition_ns_{0};
  std::atomic<std::uint64_t> last_a_ns_{0};
  std::atomic<std::uint64_t> last_b_ns_{0};
};

}  // namespace my_epuck_project_cpp
