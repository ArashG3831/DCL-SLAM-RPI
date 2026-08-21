#include "my_epuck_project_cpp/quadrature_decoder.hpp"

#include <stdexcept>

namespace my_epuck_project_cpp {

QuadratureDecoder::QuadratureDecoder(std::uint8_t initial_state, int encoder_sign)
: encoder_sign_(encoder_sign), state_(initial_state & 0x3U)
{
  if (initial_state > 3U || (encoder_sign != 1 && encoder_sign != -1)) {
    throw std::invalid_argument("invalid quadrature decoder construction arguments");
  }
}

int QuadratureDecoder::transition_delta(std::uint8_t previous, std::uint8_t current) const
{
  if (previous == current) return 0;
  if ((previous == 0 && current == 1) ||
      (previous == 1 && current == 3) ||
      (previous == 3 && current == 2) ||
      (previous == 2 && current == 0)) {
    return encoder_sign_;
  }
  if ((previous == 0 && current == 2) ||
      (previous == 2 && current == 3) ||
      (previous == 3 && current == 1) ||
      (previous == 1 && current == 0)) {
    return -encoder_sign_;
  }
  return 99;  // impossible two-bit jump
}

int QuadratureDecoder::process_state(std::uint8_t new_state, std::uint64_t timestamp_ns)
{
  if (new_state > 3U) throw std::invalid_argument("quadrature state must be 0..3");
  std::uint8_t previous = state_.exchange(new_state, std::memory_order_acq_rel);
  const int delta = transition_delta(previous, new_state);
  last_delta_.store(0, std::memory_order_relaxed);
  if (delta == 99) {
    invalid_.fetch_add(1, std::memory_order_relaxed);
    direction_.store(0, std::memory_order_relaxed);
    return 0;
  }
  if (delta == 0) return 0;  // duplicate notification: neither valid nor invalid
  count_.fetch_add(delta, std::memory_order_relaxed);
  valid_.fetch_add(1, std::memory_order_relaxed);
  last_delta_.store(static_cast<std::int8_t>(delta), std::memory_order_relaxed);
  direction_.store(delta > 0 ? 1 : -1, std::memory_order_relaxed);
  last_transition_ns_.store(timestamp_ns, std::memory_order_relaxed);
  return delta;
}

void QuadratureDecoder::initialize_state(std::uint8_t state)
{
  if (state > 3U) throw std::invalid_argument("quadrature state must be 0..3");
  state_.store(state, std::memory_order_release);
}

int QuadratureDecoder::process_edge(char channel, std::uint8_t level, std::uint64_t timestamp_ns)
{
  if ((channel != 'A' && channel != 'B') || level > 1U) {
    throw std::invalid_argument("invalid quadrature edge");
  }
  if (channel == 'A') {
    a_edges_.fetch_add(1, std::memory_order_relaxed);
    last_a_ns_.store(timestamp_ns, std::memory_order_relaxed);
  } else {
    b_edges_.fetch_add(1, std::memory_order_relaxed);
    last_b_ns_.store(timestamp_ns, std::memory_order_relaxed);
  }

  // The alert callback for a GPIO line is serialized by lgpio. A compare/exchange
  // still prevents a concurrent snapshot or callback from creating a torn state.
  std::uint8_t old = state_.load(std::memory_order_acquire);
  for (;;) {
    const std::uint8_t next = channel == 'A'
      ? static_cast<std::uint8_t>((old & 0x1U) | (level << 1U))
      : static_cast<std::uint8_t>((old & 0x2U) | level);
    if (state_.compare_exchange_weak(old, next, std::memory_order_acq_rel)) {
      const int delta = transition_delta(old, next);
      last_delta_.store(0, std::memory_order_relaxed);
      if (delta == 99) {
        invalid_.fetch_add(1, std::memory_order_relaxed);
        direction_.store(0, std::memory_order_relaxed);
        return 0;
      }
      if (delta == 0) return 0;
      count_.fetch_add(delta, std::memory_order_relaxed);
      valid_.fetch_add(1, std::memory_order_relaxed);
      last_delta_.store(static_cast<std::int8_t>(delta), std::memory_order_relaxed);
      direction_.store(delta > 0 ? 1 : -1, std::memory_order_relaxed);
      last_transition_ns_.store(timestamp_ns, std::memory_order_relaxed);
      return delta;
    }
  }
}

EncoderSnapshot QuadratureDecoder::snapshot() const
{
  const auto state = state_.load(std::memory_order_acquire);
  EncoderSnapshot s;
  s.count = count_.load(std::memory_order_relaxed);
  s.valid_transition_count = valid_.load(std::memory_order_relaxed);
  s.invalid_transition_count = invalid_.load(std::memory_order_relaxed);
  s.a_edge_count = a_edges_.load(std::memory_order_relaxed);
  s.b_edge_count = b_edges_.load(std::memory_order_relaxed);
  s.last_delta = last_delta_.load(std::memory_order_relaxed);
  s.decoded_direction = direction_.load(std::memory_order_relaxed);
  s.last_transition_time_ns = last_transition_ns_.load(std::memory_order_relaxed);
  s.last_a_edge_time_ns = last_a_ns_.load(std::memory_order_relaxed);
  s.last_b_edge_time_ns = last_b_ns_.load(std::memory_order_relaxed);
  s.a_level = static_cast<std::uint8_t>((state >> 1U) & 1U);
  s.b_level = static_cast<std::uint8_t>(state & 1U);
  s.current_state = state;
  return s;
}

void QuadratureDecoder::reset_counts()
{
  count_.store(0, std::memory_order_relaxed);
  valid_.store(0, std::memory_order_relaxed);
  invalid_.store(0, std::memory_order_relaxed);
  a_edges_.store(0, std::memory_order_relaxed);
  b_edges_.store(0, std::memory_order_relaxed);
  last_delta_.store(0, std::memory_order_relaxed);
  direction_.store(0, std::memory_order_relaxed);
  last_transition_ns_.store(0, std::memory_order_relaxed);
  last_a_ns_.store(0, std::memory_order_relaxed);
  last_b_ns_.store(0, std::memory_order_relaxed);
}

}  // namespace my_epuck_project_cpp
