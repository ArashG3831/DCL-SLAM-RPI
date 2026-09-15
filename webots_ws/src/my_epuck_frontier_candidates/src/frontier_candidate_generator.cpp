#include <algorithm>
#include <atomic>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cerrno>
#include <fcntl.h>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <signal.h>
#include <string>
#include <sys/file.h>
#include <thread>
#include <unistd.h>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <my_epuck_interfaces/msg/distributed_exploration_status.hpp>
#include <my_epuck_interfaces/msg/frontier_candidate.hpp>
#include <my_epuck_interfaces/msg/frontier_candidate_array.hpp>
#include <my_epuck_interfaces/msg/relative_pose_hypothesis.hpp>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include "frontier_exploration_ros2/frontier_explorer_core.hpp"
#include "my_epuck_frontier_candidates/candidate_utils.hpp"

using namespace std::chrono_literals;

namespace my_epuck_frontier_candidates {

namespace {

// The physical frontier core predates the optional diagnostic-only `cells`
// member.  Keep the migrated generator source-compatible with both core API
// shapes without changing frontier discovery, scoring, or dispatch behavior.
template<typename Region>
auto append_region_cells_json(std::ostream & output, const Region & region, int)
  -> decltype(region.cells.size(), void())
{
  for (std::size_t i = 0; i < region.cells.size(); ++i) {
    if (i) {output << ',';}
    output << '[' << region.cells[i].first << ',' << region.cells[i].second << ']';
  }
}

template<typename Region>
void append_region_cells_json(std::ostream &, const Region &, long)
{
  // The physical core exposes the equivalent scalar region.size field.
}

template<typename Region>
auto append_region_cells_fingerprint(std::ostringstream & output, const Region & region, int)
  -> decltype(region.cells.size(), void())
{
  for (const auto & cell : region.cells) {
    output << cell.first << ':' << cell.second << ';';
  }
}

template<typename Region>
void append_region_cells_fingerprint(std::ostringstream & output, const Region & region, long)
{
  // Keep the fallback fingerprint deterministic when exact cell membership is
  // unavailable from the physical frontier-core API.
  output << "cell-membership-unavailable:" << region.size << ';';
}

}  // namespace

class Generator : public rclcpp::Node {
  using Action = nav2_msgs::action::ComputePathToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<Action>;
  static constexpr std::chrono::duration<double> kLocalContextTolerance{0.500};

  enum class TimingSection {
    START_CYCLE,
    CORE_FRONTIER_SNAPSHOT,
    MAKE_WORK,
    FRONTIER_FILTERING,
    QUERY_RESULT_PROCESSING,
    PUBLISH_BATCH,
    COUNT
  };

  struct TimingStats {
    uint64_t calls{0};
    double total_wall_s{0.0};
    double max_wall_s{0.0};

    void add(double duration_s)
    {
      ++calls;
      total_wall_s += duration_s;
      max_wall_s = std::max(max_wall_s, duration_s);
    }
  };

  struct CycleStats {
    uint64_t cycles{0};
    uint64_t map_cells_total{0};
    uint64_t raw_frontiers_total{0};
    uint64_t final_candidates_total{0};

    void add(std::size_t map_cells, uint32_t raw_frontiers, std::size_t final_candidates)
    {
      ++cycles;
      map_cells_total += static_cast<uint64_t>(map_cells);
      raw_frontiers_total += raw_frontiers;
      final_candidates_total += static_cast<uint64_t>(final_candidates);
    }
  };

  struct TimingScope {
    Generator * owner;
    TimingSection section;
    double sim_time_s;
    std::chrono::steady_clock::time_point started;

    TimingScope(Generator * node, TimingSection value, double sim_time)
    : owner(node), section(value), sim_time_s(sim_time),
      started(std::chrono::steady_clock::now()) {}

    ~TimingScope()
    {
      if (owner) {
        owner->record_timing(
          section, sim_time_s,
          std::chrono::duration<double>(
            std::chrono::steady_clock::now() - started).count());
      }
    }
  };

  struct QueryBoundaryScope {
    Generator * owner;
    const char * exit_event;

    QueryBoundaryScope(Generator * node, const char * event)
    : owner(node), exit_event(event) {}

    ~QueryBoundaryScope()
    {
      if (owner) {
        owner->log_query_boundary(exit_event);
      }
    }
  };

  enum class State {WAITING_FOR_INPUTS, IDLE, EXTRACTING, PATH_CHECKING, PUBLISHING};

  struct Work {
    frontier_exploration_ros2::FrontierCandidate region;
    uint64_t id{0};
    double frontier_length{0.0};
    double gain{0.0};
    double euclid{0.0};
    double heading{0.0};
    std::array<double, 4> bounds{};
    geometry_msgs::msg::PoseStamped pose;
    double path{0.0};
    double score{0.0};
    uint32_t mrtsp_route_rank{std::numeric_limits<uint32_t>::max()};
    uint64_t mrtsp_route_generation{0};
    std::vector<geometry_msgs::msg::Point> path_samples;
    uint64_t map_context{0};
    uint64_t cost_context{0};
    uint64_t path_map_context{0};
    uint64_t path_cost_context{0};
    uint64_t cycles_not_queried{0};
    int64_t last_query_ns{0};
    bool path_refresh{false};
    bool tier1_unqueried{false};
    uint64_t query_event_id{0};
  };

  struct RegionDiagnostic {
    uint64_t id{0};
    frontier_exploration_ros2::FrontierCandidate region;
    std::string status{"DETECTED_NOT_QUERIED"};
    double approach_x{0.0};
    double approach_y{0.0};
    bool has_approach{false};
    double visible_reveal_gain{std::numeric_limits<double>::quiet_NaN()};
    double optimistic_cost_lower_bound_s{std::numeric_limits<double>::quiet_NaN()};
  };

  struct IdentityReference {
    uint64_t id{0};
    double centroid_x{0.0};
    double centroid_y{0.0};
    std::array<double, 4> bounds{};
  };

  struct EvaluationCache {
    uint64_t geometry_id{0};
    uint64_t map_context{0};
    uint64_t cost_context{0};
    uint64_t path_map_context{0};
    uint64_t path_cost_context{0};
    std::string classification;
    Work work;
    bool has_work{false};
    uint64_t query_count{0};
    uint64_t cycles_seen{0};
    uint64_t cycles_not_queried{0};
    int64_t last_query_ns{0};
    int64_t last_seen_ns{0};
    // Diagnostic-only record of the most recent completed query outcome.
    // This is never consulted by scheduling or classification decisions.
    std::string last_query_result;
  };

  static const char * query_termination_name(const std::string & reason)
  {
    return reason.empty() ? "OTHER" : reason.c_str();
  }

  struct Suppression {
    uint64_t revision{0};
    int64_t expires_ns{0};
  };

public:
  Generator()
  : Node("frontier_candidate_generator"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
#define P(T, N, D) N##_ = declare_parameter<T>(#N, D)
    P(std::string, robot_id, "");
    P(std::string, map_topic, "shared_map");
    P(std::string, global_costmap_topic, "global_costmap/costmap");
    P(std::string, global_frame, "shared_map");
    P(std::string, robot_base_frame, "base_footprint");
    P(std::string, compute_path_action, "compute_path_to_pose");
    P(std::string, candidate_topic, "frontier_candidates");
    P(std::string, marker_topic, "frontier_candidate_markers");
    P(std::string, path_query_lock_path, "");
    // Occupancy grids are large fragmented samples.  The generator only needs
    // the newest complete map; deployments may select best-effort/volatile
    // KeepLast(1) to avoid retaining reliable-fragment state on constrained
    // transports.  The publisher remains reliable and the final dispatch gate
    // still validates fresh costmap/path data independently.
    P(std::string, grid_subscription_reliability, "reliable");
    P(std::string, grid_subscription_durability, "transient_local");
    P(double, processing_rate_hz, .5);
    P(bool, handoff_gated, false);
    P(bool, stop_after_handoff, false);
    P(bool, event_driven_costing, false);
    // Physical solo fallback only. The simulator/cooperative default remains
    // the authoritative costmap-based search when this is false.
    P(bool, require_known_approach, false);
    P(int, occupied_threshold, 50);
    P(int, costmap_blocked_threshold, 1);
    // Use the pinned upstream decision-map pipeline as a private frontier
    // decision view.  These remain node parameters so the integration can be
    // compared against the raw path without duplicating upstream filtering.
    P(bool, frontier_map_optimization_enabled, true);
    P(double, sigma_s, 2.0);
    P(double, sigma_r, 30.0);
    P(int, dilation_kernel_radius_cells, 1);
    P(int, minimum_frontier_cells, 5);
    P(double, minimum_frontier_length_m, .05);
    P(double, stable_id_quantization_m, .05);
    P(double, approach_clearance_m, .06);
    P(double, frontier_goal_stepback_m, 0.0);
    P(double, planner_tolerance_m, .5);
    P(double, minimum_robot_distance_m, .08);
    P(int, maximum_candidates_before_path_check, 8);
    P(int, maximum_path_queries_per_cycle, 8);
    P(double, path_query_timeout_s, 1.0);
    P(std::string, planner_id, "GridBased");
    P(std::string, selection_policy, "frontier_mrtsp");
    P(double, gain_weight, 1.0);
    P(double, distance_weight, 1.0);
    P(double, path_weight, 1.0);
    P(double, heading_weight, .2);
    // These are the frozen production RPP reference limits.  They scale the
    // cost-only policy physically; they are not an ETA or an RPP simulator.
    P(double, cost_only_reference_linear_speed_mps, .13);
    P(double, cost_only_reference_angular_speed_radps, .35);
    // The upstream core remains a route/preference engine only.  The
    // distributed coordinator is still the sole owner of NavigateToPose.
    P(bool, upstream_route_ordering_enabled, false);
    P(std::string, upstream_mrtsp_solver, "dp");
    P(int, upstream_mrtsp_candidate_limit, 8);
    P(int, upstream_mrtsp_planning_horizon, 5);
    P(double, unreachable_suppression_s, 7.0);
    P(int, maximum_suppression_records, 128);
    P(double, goal_tolerance_m, .03);
    P(double, visible_gain_range_m, 11.98);
    P(double, visible_gain_fov_deg, 360.0);
    P(double, visible_gain_ray_step_deg, 2.0);
    // Classification validity is local to the frontier/approach neighborhood.
    // Exact bid-path validity remains stricter via path_context_radius_m.
    P(double, classification_context_radius_m, .35);
    P(double, path_context_radius_m, .75);
    P(bool, forensic_clearance_cells, false);
    P(bool, diagnostic_frontier_capture, false);
    diagnostic_timing_ = std::getenv("MY_EPUCK_FRONTIER_CANDIDATE_TIMING") != nullptr;
    // Compatibility marker: GENERATOR_APPROACH_CLEARANCE_CELLS and
    // P(bool,forensic_clearance_cells,false) document the opt-in evidence path.
    // The upstream core owns frontier discovery; this cache is only for the
    // project-specific asynchronous path evidence and is deliberately bounded.
    P(int, maximum_evaluation_records, 256);
#undef P
    if (robot_id_.empty()) {
      throw std::runtime_error("robot_id must be configured");
    }
    if (selection_policy_ != "frontier_cost_only" &&
        selection_policy_ != "frontier_mrtsp") {
      throw std::runtime_error(
        "selection_policy must be frontier_cost_only or frontier_mrtsp");
    }
    if (!std::isfinite(cost_only_reference_linear_speed_mps_) ||
        cost_only_reference_linear_speed_mps_ <= 0.0 ||
        !std::isfinite(cost_only_reference_angular_speed_radps_) ||
        cost_only_reference_angular_speed_radps_ <= 0.0) {
      throw std::runtime_error(
        "cost-only reference speeds must be finite and positive");
    }
    if (path_query_lock_path_.empty()) {
      path_query_lock_path_ = "/tmp/my_epuck_" + robot_id_ + "_compute_path.lock";
    }
    path_priority_path_ = path_query_lock_path_ + ".fallback_priority";
    auto grid_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    if (grid_subscription_durability_ == "volatile") {
      grid_qos.durability_volatile();
    } else if (grid_subscription_durability_ == "transient_local") {
      grid_qos.transient_local();
    } else {
      RCLCPP_WARN(
        get_logger(),
        "invalid grid_subscription_durability=%s; using transient_local",
        grid_subscription_durability_.c_str());
      grid_qos.transient_local();
    }
    if (grid_subscription_reliability_ == "reliable") {
      grid_qos.reliable();
    } else if (grid_subscription_reliability_ == "best_effort") {
      grid_qos.best_effort();
    } else {
      RCLCPP_WARN(
        get_logger(),
        "invalid grid_subscription_reliability=%s; using reliable",
        grid_subscription_reliability_.c_str());
      grid_qos.reliable();
    }
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, grid_qos,
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr message) {map_cb(message);});
    cost_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      global_costmap_topic_, grid_qos,
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr message) {cost_cb(message);});
    pub_ = create_publisher<my_epuck_interfaces::msg::FrontierCandidateArray>(
      candidate_topic_, rclcpp::QoS(1).reliable());
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      marker_topic_, rclcpp::QoS(1).reliable());
    initialize_upstream_core();
    planner_ = rclcpp_action::create_client<Action>(this, compute_path_action_);
    if (event_driven_costing_) {
      auto status_qos = rclcpp::QoS(rclcpp::KeepLast(1));
      status_qos.reliable().transient_local();
      coordinator_status_subscription_ =
        create_subscription<my_epuck_interfaces::msg::DistributedExplorationStatus>(
        "/" + robot_id_ + "/distributed_status", status_qos,
        [this](my_epuck_interfaces::msg::DistributedExplorationStatus::ConstSharedPtr message) {
          coordinator_status_cb(message);
        });
    }
    if (handoff_gated_ || stop_after_handoff_) {
      handoff_subscription_ = create_subscription<my_epuck_interfaces::msg::RelativePoseHypothesis>(
        "/cslam/relative_pose/hypotheses", rclcpp::QoS(1).reliable(),
        [this](my_epuck_interfaces::msg::RelativePoseHypothesis::ConstSharedPtr message) {
          if (!message->accepted || message->status != "ACCEPTED") {
            return;
          }
          if (handoff_gated_ && !processing_active_) {
            processing_active_ = true;
            timer_ = create_wall_timer(
              std::chrono::duration<double>(1.0 / std::max(.01, processing_rate_hz_)),
              [this] {tick();});
            receipt_summary_timer_ = create_wall_timer(
              1s, [this] {emit_receipt_summary();});
            RCLCPP_INFO(get_logger(), "FRONTIER_PHASE post_handoff=true processing_active=true");
          } else if (stop_after_handoff_ && processing_active_) {
            cycle_termination_reason_ = "HANDOFF_CANCEL";
            processing_active_ = false;
            request_generation_++;
            active_request_ = 0;
            if (timer_) {timer_->cancel();}
            cancel_query_timeout();
            if (retry_timer_) {retry_timer_->cancel();}
            if (active_) {
              planner_->async_cancel_goal(active_);
              active_.reset();
            }
            release_path_lock();
            state_ = State::IDLE;
            works_.clear();
            reachable_.clear();
            cycle_map_.reset();
            cycle_cost_.reset();
            RCLCPP_INFO(
              get_logger(),
              "FRONTIER_PHASE pre_handoff_stopped=true reason=ACCEPTED_HANDOFF");
          }
        });
      if (stop_after_handoff_ && !handoff_gated_) {
        processing_active_ = true;
        timer_ = create_wall_timer(
          std::chrono::duration<double>(1.0 / std::max(.01, processing_rate_hz_)),
          [this] {tick();});
      }
    } else {
      processing_active_ = true;
      timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / std::max(.01, processing_rate_hz_)),
        [this] {tick();});
    }
    if (!handoff_gated_) {
      receipt_summary_timer_ = create_wall_timer(1s, [this] {emit_receipt_summary();});
    }
    RCLCPP_INFO(
      get_logger(),
      "candidate generator: persistent fair frontier evaluation policy=%s map=%s costmap=%s planner=%s budget=%d handoff_gated=%s",
      selection_policy_.c_str(),
      map_topic_.c_str(), global_costmap_topic_.c_str(), compute_path_action_.c_str(),
      maximum_path_queries_per_cycle_, handoff_gated_ ? "true" : "false");
    RCLCPP_INFO(
      get_logger(),
      "COST_ONLY_MOTION_REFERENCES linear_mps=%.6f angular_radps=%.6f",
      cost_only_reference_linear_speed_mps_, cost_only_reference_angular_speed_radps_);
    RCLCPP_INFO(
      get_logger(), "GENERATOR_GRID_QOS reliability=%s durability=%s depth=1",
      grid_subscription_reliability_.c_str(), grid_subscription_durability_.c_str());
  }

  ~Generator() override
  {
    emit_timing_summary();
    request_generation_++;
    active_request_ = 0;
    cancel_query_timeout();
    if (retry_timer_) {retry_timer_->cancel();}
    release_path_lock();
    if (active_) {planner_->async_cancel_goal(active_);}
  }

private:
  static const char * timing_section_name(TimingSection section)
  {
    switch (section) {
      case TimingSection::START_CYCLE: return "start_cycle";
      case TimingSection::CORE_FRONTIER_SNAPSHOT: return "get_frontier_snapshot";
      case TimingSection::MAKE_WORK: return "make_work";
      case TimingSection::FRONTIER_FILTERING: return "frontier_filtering";
      case TimingSection::QUERY_RESULT_PROCESSING: return "query_result_processing";
      case TimingSection::PUBLISH_BATCH: return "publish_batch";
      default: return "unknown";
    }
  }

  static int timing_window(double sim_time_s)
  {
    if (sim_time_s >= 50.0 && sim_time_s < 100.0) {return 0;}
    if (sim_time_s >= 280.0 && sim_time_s <= 330.0) {return 1;}
    return -1;
  }

  void record_timing(TimingSection section, double sim_time_s, double duration_s)
  {
    if (!diagnostic_timing_) {return;}
    const int window = timing_window(sim_time_s);
    if (window < 0) {return;}
    timing_stats_[static_cast<std::size_t>(section)][static_cast<std::size_t>(window)].add(
      duration_s);
  }

  void record_cycle(std::size_t map_cells, uint32_t raw_frontiers, std::size_t final_candidates)
  {
    if (!diagnostic_timing_) {return;}
    const double sim_time_s = now().seconds();
    const int window = timing_window(sim_time_s);
    if (window >= 0) {
      cycle_stats_[static_cast<std::size_t>(window)].add(
        map_cells, raw_frontiers, final_candidates);
    }
  }

  void emit_timing_summary()
  {
    if (!diagnostic_timing_ || timing_summary_emitted_) {return;}
    timing_summary_emitted_ = true;
    constexpr const char * windows[] = {"early", "late"};
    for (std::size_t window = 0; window < 2; ++window) {
      RCLCPP_WARN(
        get_logger(),
        "FRONTIER_GENERATOR_CYCLE_SUMMARY robot=%s window=%s cycles=%lu map_cells_total=%lu raw_frontiers_total=%lu final_candidates_total=%lu",
        robot_id_.c_str(), windows[window], cycle_stats_[window].cycles,
        cycle_stats_[window].map_cells_total, cycle_stats_[window].raw_frontiers_total,
        cycle_stats_[window].final_candidates_total);
      for (std::size_t section = 0;
        section < static_cast<std::size_t>(TimingSection::COUNT); ++section)
      {
        const auto & stats = timing_stats_[section][window];
        const double mean = stats.calls == 0 ? 0.0 : stats.total_wall_s / stats.calls;
        RCLCPP_WARN(
          get_logger(),
          "FRONTIER_GENERATOR_TIMING robot=%s window=%s section=%s calls=%lu total_wall_s=%.9f mean_wall_s=%.9f max_wall_s=%.9f",
          robot_id_.c_str(), windows[window],
          timing_section_name(static_cast<TimingSection>(section)), stats.calls,
          stats.total_wall_s, mean, stats.max_wall_s);
      }
    }
    RCLCPP_WARN(
      get_logger(),
      "FRONTIER_GENERATOR_LEASE_SUMMARY robot=%s acquire_attempts=%lu retry_count=%lu wait_total_wall_s=%.9f wait_max_wall_s=%.9f hold_count=%lu hold_total_wall_s=%.9f hold_max_wall_s=%.9f priority_yields=%lu",
      robot_id_.c_str(), path_lock_acquire_attempts_, path_lock_retry_count_,
      path_lock_wait_total_s_, path_lock_wait_max_s_, path_lock_hold_count_,
      path_lock_hold_total_s_, path_lock_hold_max_s_, path_priority_yields_);
  }

  static const char * action_result_name(rclcpp_action::ResultCode code)
  {
    switch (code) {
      case rclcpp_action::ResultCode::SUCCEEDED: return "SUCCEEDED";
      case rclcpp_action::ResultCode::CANCELED: return "CANCELED";
      case rclcpp_action::ResultCode::ABORTED: return "ABORTED";
      default: return "UNKNOWN";
    }
  }

  struct CostmapValidationEvidence {
    int goal_cost{-1};
    int start_cost{-1};
    double nearest_lethal_m{std::numeric_limits<double>::quiet_NaN()};
    double nearest_inflated_m{std::numeric_limits<double>::quiet_NaN()};
    double nearest_unknown_m{std::numeric_limits<double>::quiet_NaN()};
    bool goal_lethal{false};
    bool goal_inflated{false};
  };

  static CostmapValidationEvidence costmap_validation_evidence(
    const frontier_exploration_ros2::OccupancyGrid2d * costmap,
    double start_x, double start_y, double goal_x, double goal_y,
    double radius_m)
  {
    CostmapValidationEvidence evidence;
    if (!costmap) {
      return evidence;
    }
    int start_x_cell, start_y_cell, goal_x_cell, goal_y_cell;
    if (costmap->worldToMapNoThrow(start_x, start_y, start_x_cell, start_y_cell)) {
      evidence.start_cost = costmap->getCost(start_x_cell, start_y_cell);
    }
    if (!costmap->worldToMapNoThrow(goal_x, goal_y, goal_x_cell, goal_y_cell)) {
      return evidence;
    }
    evidence.goal_cost = costmap->getCost(goal_x_cell, goal_y_cell);
    evidence.goal_lethal = evidence.goal_cost >= 254;
    evidence.goal_inflated = evidence.goal_cost > 0 && evidence.goal_cost < 254;
    const int radius_cells = std::max(
      1, static_cast<int>(std::ceil(radius_m / costmap->map().info.resolution)));
    for (int y = goal_y_cell - radius_cells; y <= goal_y_cell + radius_cells; ++y) {
      for (int x = goal_x_cell - radius_cells; x <= goal_x_cell + radius_cells; ++x) {
        if (x < 0 || y < 0 || x >= costmap->getSizeX() || y >= costmap->getSizeY()) {
          continue;
        }
        const auto value = costmap->getCost(x, y);
        const double distance = std::hypot(
          static_cast<double>(x - goal_x_cell) * costmap->map().info.resolution,
          static_cast<double>(y - goal_y_cell) * costmap->map().info.resolution);
        if (distance > radius_m) {
          continue;
        }
        if (value < 0) {
          evidence.nearest_unknown_m = std::min(evidence.nearest_unknown_m, distance);
        } else if (value >= 254) {
          evidence.nearest_lethal_m = std::min(evidence.nearest_lethal_m, distance);
        } else if (value > 0) {
          evidence.nearest_inflated_m = std::min(evidence.nearest_inflated_m, distance);
        }
      }
    }
    return evidence;
  }

  void log_path_validation(
    const Work & candidate, const GoalHandle::WrappedResult & result,
    const std::optional<double> & length, uint64_t candidate_generation,
    double duration_s, const nav_msgs::msg::Path * path_override = nullptr) const
  {
    if (!diagnostic_frontier_capture_ ||
      result.code != rclcpp_action::ResultCode::SUCCEEDED || !result.result ||
      result.result->error_code != Action::Result::NONE || length)
    {
      return;
    }
    const auto & path = path_override ? *path_override : result.result->path;
    const std::size_t pose_count = path.poses.size();
    std::vector<std::size_t> nonfinite_indices;
    for (std::size_t index = 0; index < path.poses.size(); ++index) {
      const auto & pose = path.poses[index];
      const auto & p = pose.pose.position;
      const auto & q = pose.pose.orientation;
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
        !std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) ||
        !std::isfinite(q.w))
      {
        nonfinite_indices.push_back(index);
      }
    }
    const auto endpoint = [&](std::size_t index) {
      if (path.poses.empty()) {
        return std::array<double, 3>{
          std::numeric_limits<double>::quiet_NaN(),
          std::numeric_limits<double>::quiet_NaN(),
          std::numeric_limits<double>::quiet_NaN()};
      }
      const auto & pose = path.poses[index].pose;
      return std::array<double, 3>{
        pose.position.x, pose.position.y, orientation_yaw(pose.orientation)};
    };
    const auto first = endpoint(0);
    const auto last = endpoint(path.poses.empty() ? 0 : path.poses.size() - 1);
    const double final_goal_distance = !path.poses.empty() ? std::hypot(
      last[0] - candidate.pose.pose.position.x,
      last[1] - candidate.pose.pose.position.y) :
      std::numeric_limits<double>::quiet_NaN();
    const double start_goal_distance = std::hypot(
      rx_ - candidate.pose.pose.position.x, ry_ - candidate.pose.pose.position.y);
    std::string reason;
    if (pose_count == 0) {
      reason = "EMPTY_PATH";
    } else if (!nonfinite_indices.empty()) {
      reason = "NONFINITE_POSE";
    } else if (pose_count == 1 && start_goal_distance > goal_tolerance_m_) {
      reason = "ONE_POSE_PATH";
    } else if (final_goal_distance > goal_tolerance_m_) {
      reason = "FINAL_POSE_TOLERANCE";
    } else if (length) {
      reason = "VALID";
    } else {
      reason = "OTHER_VALIDATION_FAILURE";
    }
    std::optional<frontier_exploration_ros2::OccupancyGrid2d> cost_grid;
    if (cycle_cost_) {
      cost_grid.emplace(cycle_cost_);
    }
    const auto costmap = costmap_validation_evidence(
      cost_grid ? &*cost_grid : nullptr, rx_, ry_,
      candidate.pose.pose.position.x, candidate.pose.pose.position.y, 0.5);
    std::ostringstream nonfinite_text;
    if (nonfinite_indices.empty()) {
      nonfinite_text << "none";
    } else {
      for (std::size_t i = 0; i < nonfinite_indices.size(); ++i) {
        if (i) {nonfinite_text << ',';}
        nonfinite_text << nonfinite_indices[i];
      }
    }
    const auto finite_or_na = [](double value) {
      return std::isfinite(value) ? std::to_string(value) : std::string("nan");
    };
    RCLCPP_INFO(
      get_logger(),
      "FRONTIER_PATH_VALIDATION robot=%s sim_time=%.9f query_id=%lu id=%lu "
      "candidate_generation_id=%lu planner_id=%s planner_frame=%s goal_frame=%s "
      "path_frame=%s map_revision=%lu map_stamp_s=%.9f global_costmap_revision=%lu "
      "global_costmap_stamp_s=%.9f local_costmap_revision=UNAVAILABLE "
      "local_costmap_stamp_s=UNAVAILABLE action_result=%s error_code=%d error_name=%s "
      "error_message=\"%s\" duration_s=%.6f start_x=%.9f start_y=%.9f start_yaw=%.9f "
      "goal_x=%.9f goal_y=%.9f goal_yaw=%.9f "
      "path_pose_count=%zu first_x=%.9f first_y=%.9f first_yaw=%.9f "
      "last_x=%.9f last_y=%.9f last_yaw=%.9f final_goal_distance_m=%.9f "
      "validator_path_length_m=%.9f nonfinite=%s nonfinite_indices=%s "
      "global_start_cost=%d global_goal_cost=%d global_goal_lethal=%s "
      "global_goal_inflated=%s global_nearest_lethal_m=%s "
      "global_nearest_inflated_m=%s global_nearest_unknown_m=%s "
      "planner_result_metadata=\"%s\" validation=%s",
      robot_id_.c_str(), now().seconds(), candidate.query_event_id, candidate.id,
      candidate_generation, planner_id_.c_str(), candidate.pose.header.frame_id.c_str(),
      candidate.pose.header.frame_id.c_str(), path.poses.empty() ? "UNAVAILABLE" :
      path.poses.front().header.frame_id.c_str(), cycle_revision_,
      cycle_map_ ? stamp_seconds(cycle_map_->header.stamp) : 0.0, cycle_cost_revision_,
      cycle_cost_ ? stamp_seconds(cycle_cost_->header.stamp) : 0.0,
      action_result_name(result.code), result.result->error_code,
      nav2_error_name(result.result->error_code), log_safe(result.result->error_msg).c_str(),
      duration_s, rx_, ry_, yaw_, candidate.pose.pose.position.x,
      candidate.pose.pose.position.y, orientation_yaw(candidate.pose.pose.orientation),
      pose_count, first[0], first[1], first[2], last[0], last[1], last[2],
      final_goal_distance, length.value_or(-1.0), nonfinite_indices.empty() ? "false" : "true",
      nonfinite_text.str().c_str(), costmap.start_cost, costmap.goal_cost,
      costmap.goal_lethal ? "true" : "false", costmap.goal_inflated ? "true" : "false",
      finite_or_na(costmap.nearest_lethal_m).c_str(),
      finite_or_na(costmap.nearest_inflated_m).c_str(),
      finite_or_na(costmap.nearest_unknown_m).c_str(), log_safe(result.result->error_msg).c_str(),
      reason.c_str());
  }

  static const char * nav2_error_name(int error_code)
  {
    switch (error_code) {
      case Action::Result::NONE: return "NONE";
      case Action::Result::UNKNOWN: return "UNKNOWN";
      case Action::Result::INVALID_PLANNER: return "INVALID_PLANNER";
      case Action::Result::TF_ERROR: return "TF_ERROR";
      case Action::Result::START_OUTSIDE_MAP: return "START_OUTSIDE_MAP";
      case Action::Result::GOAL_OUTSIDE_MAP: return "GOAL_OUTSIDE_MAP";
      case Action::Result::START_OCCUPIED: return "START_OCCUPIED";
      case Action::Result::GOAL_OCCUPIED: return "GOAL_OCCUPIED";
      case Action::Result::TIMEOUT: return "TIMEOUT";
      case Action::Result::NO_VALID_PATH: return "NO_VALID_PATH";
      default: return "UNRECOGNIZED";
    }
  }

  static double stamp_seconds(const builtin_interfaces::msg::Time & stamp)
  {
    return static_cast<double>(stamp.sec) +
           static_cast<double>(stamp.nanosec) * 1.0e-9;
  }

  static double orientation_yaw(const geometry_msgs::msg::Quaternion & q)
  {
    return std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  struct StepBackPath
  {
    nav_msgs::msg::Path path;
    double length{0.0};
  };

  static std::optional<StepBackPath> step_back_path_pose(
    const nav_msgs::msg::Path & path, double stepback_m)
  {
    if (!std::isfinite(stepback_m) || stepback_m < 0.0 || path.poses.empty()) {
      return std::nullopt;
    }
    double total = 0.0;
    for (std::size_t i = 1; i < path.poses.size(); ++i) {
      const auto & a = path.poses[i - 1].pose.position;
      const auto & b = path.poses[i].pose.position;
      const double segment = std::hypot(b.x - a.x, b.y - a.y);
      if (!std::isfinite(segment)) {
        return std::nullopt;
      }
      total += segment;
    }
    if (!std::isfinite(total) || total <= stepback_m + 1e-9) {
      return std::nullopt;
    }
    if (stepback_m <= 1e-9) {
      return StepBackPath{path, total};
    }

    double remaining = stepback_m;
    for (std::size_t i = path.poses.size() - 1; i > 0; --i) {
      const auto & from = path.poses[i - 1].pose;
      const auto & to = path.poses[i].pose;
      const double segment = std::hypot(
        to.position.x - from.position.x, to.position.y - from.position.y);
      if (segment <= 1e-12) {
        continue;
      }
      if (remaining <= segment + 1e-12) {
        const double alpha = std::clamp((segment - remaining) / segment, 0.0, 1.0);
        StepBackPath result;
        result.path.header = path.header;
        result.path.poses.assign(path.poses.begin(), path.poses.begin() + i);
        auto interpolated = from;
        interpolated.position.x = from.position.x +
          alpha * (to.position.x - from.position.x);
        interpolated.position.y = from.position.y +
          alpha * (to.position.y - from.position.y);
        const double from_yaw = orientation_yaw(from.orientation);
        const double to_yaw = orientation_yaw(to.orientation);
        const double yaw_delta = std::atan2(
          std::sin(to_yaw - from_yaw), std::cos(to_yaw - from_yaw));
        const double interpolated_yaw = from_yaw + alpha * yaw_delta;
        interpolated.orientation.x = 0.0;
        interpolated.orientation.y = 0.0;
        interpolated.orientation.z = std::sin(interpolated_yaw / 2.0);
        interpolated.orientation.w = std::cos(interpolated_yaw / 2.0);
        geometry_msgs::msg::PoseStamped stamped;
        stamped.header = path.header;
        stamped.pose = interpolated;
        result.path.poses.push_back(std::move(stamped));
        result.length = total - stepback_m;
        return result;
      }
      remaining -= segment;
    }
    return std::nullopt;
  }

  static const char * query_failure_class(
    rclcpp_action::ResultCode action_result, int error_code, bool timed_out = false)
  {
    if (timed_out || error_code == Action::Result::TIMEOUT) {
      return "PLANNER_QUERY_TIMEOUT";
    }
    if (error_code == Action::Result::START_OUTSIDE_MAP ||
      error_code == Action::Result::GOAL_OUTSIDE_MAP ||
      error_code == Action::Result::START_OCCUPIED ||
      error_code == Action::Result::GOAL_OCCUPIED ||
      error_code == Action::Result::NO_VALID_PATH)
    {
      return "CANDIDATE_UNREACHABLE";
    }
    if (error_code == Action::Result::INVALID_PLANNER) {
      return "PLANNER_LIFECYCLE_UNAVAILABLE";
    }
    if (error_code == Action::Result::TF_ERROR) {
      return "TRANSIENT_PLANNER_QUERY_FAILURE";
    }
    if (action_result != rclcpp_action::ResultCode::SUCCEEDED) {
      return "TRANSIENT_PLANNER_QUERY_FAILURE";
    }
    return "TRANSIENT_PLANNER_QUERY_FAILURE";
  }

  static std::string log_safe(std::string value)
  {
    std::replace(value.begin(), value.end(), '\n', ' ');
    std::replace(value.begin(), value.end(), '\r', ' ');
    std::replace(value.begin(), value.end(), '"', '\'');
    return value;
  }

  void initialize_upstream_core()
  {
    frontier_exploration_ros2::FrontierExplorerCoreParams params;
    params.map_topic = map_topic_;
    params.costmap_topic = global_costmap_topic_;
    params.local_costmap_topic = "";
    params.navigate_to_pose_action_name = "";
    params.global_frame = global_frame_;
    params.robot_base_frame = robot_base_frame_;
    params.frontier_marker_topic = marker_topic_;
    params.frontier_map_optimization_enabled = frontier_map_optimization_enabled_;
    params.sigma_s = sigma_s_;
    params.sigma_r = sigma_r_;
    params.dilation_kernel_radius_cells = dilation_kernel_radius_cells_;
    const std::string configured_mrtsp_solver =
      upstream_route_ordering_enabled_ ? upstream_mrtsp_solver_ : "greedy";
    params.mrtsp_solver = configured_mrtsp_solver;
    params.dp_solver_candidate_limit = static_cast<std::size_t>(
      std::max(1, upstream_mrtsp_candidate_limit_));
    params.dp_planning_horizon = static_cast<std::size_t>(
      std::max(1, upstream_mrtsp_planning_horizon_));
    params.occ_threshold = occupied_threshold_;
    params.min_frontier_size_cells = minimum_frontier_cells_;
    params.frontier_candidate_min_goal_distance_m = minimum_robot_distance_m_;
    params.frontier_selection_min_distance = minimum_robot_distance_m_;
    params.map_processing_rate_hz = 0.0;
    params.frontier_suppression_enabled = false;

    frontier_exploration_ros2::FrontierExplorerCoreCallbacks callbacks;
    callbacks.now_ns = [this]() {return now().nanoseconds();};
    callbacks.get_current_pose = [this]() -> std::optional<geometry_msgs::msg::Pose> {
      geometry_msgs::msg::Pose pose;
      pose.position.x = rx_;
      pose.position.y = ry_;
      pose.orientation.w = 1.0;
      return pose;
    };
    callbacks.wait_for_action_server = [](double) {return true;};
    callbacks.dispatch_goal_request = [this](
      const frontier_exploration_ros2::GoalDispatchRequest &) {
        ++autonomous_dispatch_attempts_;
        RCLCPP_ERROR(
          get_logger(),
          "UPSTREAM_AUTONOMOUS_DISPATCH_BLOCKED attempts=%lu",
          autonomous_dispatch_attempts_);
      };
    callbacks.publish_frontier_markers = [](const frontier_exploration_ros2::FrontierSequence &) {};
    callbacks.publish_selected_frontier_pose = [](const geometry_msgs::msg::PoseStamped &) {};
    callbacks.publish_optimized_map = [](const nav_msgs::msg::OccupancyGrid &) {};
    callbacks.on_exploration_complete = []() {};
    callbacks.debug_outputs_enabled = [this]() {return diagnostic_frontier_capture_;};
    callbacks.log_debug = [this](const std::string & text) {RCLCPP_DEBUG(get_logger(), "%s", text.c_str());};
    callbacks.log_info = [this](const std::string & text) {RCLCPP_INFO(get_logger(), "%s", text.c_str());};
    callbacks.log_warn = [this](const std::string & text) {RCLCPP_WARN(get_logger(), "%s", text.c_str());};
    callbacks.log_error = [this](const std::string & text) {RCLCPP_ERROR(get_logger(), "%s", text.c_str());};
    // Leave frontier_search unset so the established upstream search and
    // accessible-goal generation remain authoritative.
    core_ = std::make_unique<frontier_exploration_ros2::FrontierExplorerCore>(
      std::move(params), std::move(callbacks));
    core_->exploration_enabled = false;
    RCLCPP_INFO(
      get_logger(),
      "UPSTREAM_FRONTIER_CORE_ACTIVE backend=frontier_exploration_ros2::FrontierExplorerCore dispatch=false map=%s costmap=%s",
      map_topic_.c_str(), global_costmap_topic_.c_str());
    RCLCPP_INFO(
      get_logger(),
      "UPSTREAM_DECISION_MAP_CONFIG optimization=%s sigma_s=%.3f sigma_r=%.3f dilation_radius_cells=%d min_frontier_cells=%d",
      frontier_map_optimization_enabled_ ? "true" : "false", sigma_s_, sigma_r_,
      dilation_kernel_radius_cells_, minimum_frontier_cells_);
    RCLCPP_INFO(
      get_logger(),
      "UPSTREAM_ROUTE_CONTEXT enabled=%s solver=%s candidate_limit=%d horizon=%d",
      upstream_route_ordering_enabled_ ? "true" : "false",
      configured_mrtsp_solver.c_str(), upstream_mrtsp_candidate_limit_,
      upstream_mrtsp_planning_horizon_);
  }

  void map_cb(nav_msgs::msg::OccupancyGrid::ConstSharedPtr message)
  {
    if ((handoff_gated_ || stop_after_handoff_) && !processing_active_) {return;}
    const auto checksum = map_checksum(*message);
    const auto receipt = std::chrono::steady_clock::now().time_since_epoch();
    std::lock_guard<std::mutex> lock(mu_);
    const bool changed = !latest_map_ || checksum != map_sum_;
    latest_map_ = message;
    ++map_receipts_;
    if (changed) {
      map_sum_ = checksum;
      ++revision_;
      ++map_changed_;
      pending_ = true;
    }
    last_map_receipt_ns_ = std::chrono::duration_cast<std::chrono::nanoseconds>(receipt).count();
  }

  void cost_cb(nav_msgs::msg::OccupancyGrid::ConstSharedPtr message)
  {
    if ((handoff_gated_ || stop_after_handoff_) && !processing_active_) {return;}
    const auto checksum = map_checksum(*message);
    const auto receipt = std::chrono::steady_clock::now().time_since_epoch();
    std::lock_guard<std::mutex> lock(mu_);
    const bool changed = !latest_cost_ || checksum != cost_sum_;
    latest_cost_ = message;
    ++cost_receipts_;
    if (changed) {
      cost_sum_ = checksum;
      ++cost_revision_;
      ++cost_changed_;
    }
    last_cost_receipt_ns_ = std::chrono::duration_cast<std::chrono::nanoseconds>(receipt).count();
  }

  void apply_pending_core_inputs(
    const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & map,
    const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & costmap,
    uint64_t map_revision, uint64_t costmap_revision)
  {
    if (!core_) {
      return;
    }
    // Apply only checksum-changing revisions from the bounded processing
    // timer.  Calling the upstream core synchronously from every reliable
    // OccupancyGrid callback repeatedly rebuilt its decision-map state and
    // invalidated its snapshot even when the map contents were unchanged.
    // Coalescing keeps the latest complete sample, lets DDS return from the
    // callback promptly, and preserves the existing timer/freshness policy.
    if (map && map_revision != core_map_revision_) {
      core_->occupancyGridCallback(frontier_exploration_ros2::OccupancyGrid2d(map));
      core_map_revision_ = map_revision;
      RCLCPP_INFO(
        get_logger(), "GENERATOR_CORE_INPUT_APPLY kind=map revision=%lu", map_revision);
    }
    if (costmap && costmap_revision != core_costmap_revision_) {
      core_->costmapCallback(frontier_exploration_ros2::OccupancyGrid2d(costmap));
      core_costmap_revision_ = costmap_revision;
      RCLCPP_INFO(
        get_logger(), "GENERATOR_CORE_INPUT_APPLY kind=costmap revision=%lu", costmap_revision);
    }
  }

  void emit_receipt_summary()
  {
    RCLCPP_INFO(
      get_logger(),
      "GENERATOR_EXECUTOR_HEARTBEAT state=%d active_request=%lu request_generation=%lu "
      "timer_owner_request=%lu retry_generation=%lu query_index=%zu queries=%zu",
      static_cast<int>(state_.load()), active_request_.load(), request_generation_.load(),
      timeout_timer_request_.load(), retry_generation_, query_index_, queries_);
    uint64_t map_receipts, map_changed, cost_receipts, cost_changed;
    uint64_t map_revision, cost_revision, map_checksum_value, cost_checksum_value;
    int64_t map_receipt_ns, cost_receipt_ns;
    {
      std::lock_guard<std::mutex> lock(mu_);
      map_receipts = map_receipts_;
      map_changed = map_changed_;
      cost_receipts = cost_receipts_;
      cost_changed = cost_changed_;
      map_revision = revision_;
      cost_revision = cost_revision_;
      map_checksum_value = map_sum_;
      cost_checksum_value = cost_sum_;
      map_receipt_ns = last_map_receipt_ns_;
      cost_receipt_ns = last_cost_receipt_ns_;
    }
    if (map_receipts == 0 && cost_receipts == 0) {return;}
    RCLCPP_INFO(
      get_logger(),
      "GENERATOR_INPUT_SUMMARY map_receipts=%lu map_changed=%lu map_revision=%lu map_checksum=%lu map_receipt_steady_ns=%ld costmap_receipts=%lu costmap_changed=%lu costmap_revision=%lu costmap_checksum=%lu costmap_receipt_steady_ns=%ld",
      map_receipts, map_changed, map_revision, map_checksum_value, map_receipt_ns,
      cost_receipts, cost_changed, cost_revision, cost_checksum_value, cost_receipt_ns);
  }

  void tick()
  {
    if (state_ == State::PATH_CHECKING || state_ == State::EXTRACTING ||
      state_ == State::PUBLISHING) {return;}
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr map, costmap;
    uint64_t map_revision, costmap_revision;
    {
      std::lock_guard<std::mutex> lock(mu_);
      map = latest_map_;
      costmap = latest_cost_;
      map_revision = revision_;
      costmap_revision = cost_revision_;
      pending_ = false;
    }
    if (!map || !costmap) {
      state_ = State::WAITING_FOR_INPUTS;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "waiting for shared map and global costmap");
      return;
    }
    if (map->header.frame_id != global_frame_ || costmap->header.frame_id != global_frame_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "map/costmap frame mismatch");
      return;
    }
    apply_pending_core_inputs(map, costmap, map_revision, costmap_revision);
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_.lookupTransform(
        global_frame_, robot_base_frame_, tf2::TimePointZero, 100ms);
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "robot pose unavailable: %s", error.what());
      return;
    }
    const auto q = transform.transform.rotation;
    const double yaw = std::atan2(
      2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    start_cycle(
      map, costmap, map_revision, costmap_revision,
      transform.transform.translation.x, transform.transform.translation.y, yaw);
  }

  bool costing_open() const
  {
    return !event_driven_costing_ || costing_epoch_open_.load();
  }

  static const char * coordinator_state_name(uint8_t state)
  {
    using Status = my_epuck_interfaces::msg::DistributedExplorationStatus;
    switch (state) {
      case Status::BIDDING: return "BIDDING";
      case Status::NAVIGATING: return "NAVIGATING";
      case Status::WAITING_FOR_TRAFFIC: return "WAITING_TRAFFIC";
      case Status::WAITING_FOR_INPUTS: return "IDLE";
      default: return "OTHER";
    }
  }

  void coordinator_status_cb(
    my_epuck_interfaces::msg::DistributedExplorationStatus::ConstSharedPtr message)
  {
    if (!message || message->source_robot_id != robot_id_) {
      return;
    }
    last_coordinator_state_ = message->state;
    if (message->state == my_epuck_interfaces::msg::DistributedExplorationStatus::NAVIGATING) {
      RCLCPP_INFO(
        get_logger(),
        "FRONTIER_COSTING_GATE event=RECEIVED_NAVIGATING epoch_id=%lu sim_time=%.6f "
        "last_alternative_request_time=%.6f active_request=%lu",
        costing_epoch_id_, now().seconds(), last_alternative_request_time_s_,
        active_request_.load());
    }
    const bool request_costing =
      !message->terminal &&
      !message->local_nav_goal_active &&
      message->state == my_epuck_interfaces::msg::DistributedExplorationStatus::BIDDING;
    const bool same_gate_state =
      status_gate_seen_ && request_costing == last_costing_request_;
    if (!request_costing && same_gate_state) {
      return;
    }
    if (request_costing && same_gate_state && costing_epoch_open_.load()) {
      // A changed current digest is ordinary decision-epoch churn.  Keep the
      // costing epoch open and let its next cycle consume the newest geometry.
      last_costing_union_hash_ = message->union_hash;
      return;
    }
    if (request_costing && same_gate_state && !costing_epoch_open_.load() &&
      message->union_hash == last_costing_union_hash_)
    {
      return;
    }
    status_gate_seen_ = true;
    last_costing_request_ = request_costing;
    last_costing_union_hash_ = message->union_hash;
    if (request_costing) {
      const std::string reason = message->reason.empty() ?
        "idle transition" : message->reason;
      if (costing_epoch_open_.load()) {
        ++request_generation_;
        active_request_ = 0;
        cancel_query_timeout();
        cancel_retry_timer();
        if (active_) {
          planner_->async_cancel_goal(active_);
          active_.reset();
        }
        release_path_lock();
        works_.clear();
        reachable_.clear();
      }
      costing_epoch_open_.store(true);
      evaluation_cache_.clear();
      works_.clear();
      reachable_.clear();
      ++costing_epoch_id_;
      last_alternative_request_time_s_ = -1.0;
      RCLCPP_INFO(
        get_logger(),
        "FRONTIER_COSTING_EPOCH event=OPEN epoch_id=%lu sim_time=%.6f reason=%s",
        costing_epoch_id_, now().seconds(), reason.c_str());
    } else {
      if (costing_epoch_open_.load()) {
        const std::string reason = message->reason.empty() ?
          (message->local_nav_goal_active ? "goal active" : "idle transition") :
          message->reason;
        RCLCPP_INFO(
          get_logger(),
          "FRONTIER_COSTING_EPOCH event=CLOSE epoch_id=%lu sim_time=%.6f reason=%s "
          "last_alternative_request_time=%.6f active_request=%lu",
          costing_epoch_id_, now().seconds(), reason.c_str(),
          last_alternative_request_time_s_, active_request_.load());
      }
      costing_epoch_open_.store(false);
      ++request_generation_;
      active_request_ = 0;
      cancel_query_timeout();
      cancel_retry_timer();
      if (active_) {
        planner_->async_cancel_goal(active_);
        active_.reset();
      }
      release_path_lock();
      works_.clear();
      reachable_.clear();
      if (state_ == State::PATH_CHECKING) {
        state_ = State::IDLE;
      }
    }
  }

  Work make_work(
    const frontier_exploration_ros2::FrontierCandidate & region,
    uint64_t id, const frontier_exploration_ros2::OccupancyGrid2d & map,
    const frontier_exploration_ros2::OccupancyGrid2d & costmap,
    double rx, double ry, double yaw, int64_t last_query_ns, uint64_t cycles_not_queried)
  {
    Work work;
    work.region = region;
    work.id = id;
    work.frontier_length = static_cast<double>(region.size) * map.map().info.resolution;
    work.bounds = frontier_world_bounds(region, map);
    const auto inward_margin =
      planner_tolerance_m_ + .5 * costmap.map().info.resolution + 1e-6;
    const auto safe = require_known_approach_ ? find_safe_approach_known_free(
      map, costmap, region.goal_point->first, region.goal_point->second,
      region.centroid.first, region.centroid.second, .75, inward_margin,
      approach_clearance_m_, occupied_threshold_, costmap_blocked_threshold_) :
      find_safe_approach(
        costmap, region.goal_point->first, region.goal_point->second,
        region.centroid.first, region.centroid.second, .75, inward_margin,
        approach_clearance_m_, costmap_blocked_threshold_);
    if (!safe) {return work;}
    const auto world = costmap.mapToWorld(safe->first, safe->second);
    work.pose.header.frame_id = global_frame_;
    work.pose.header.stamp = now();
    work.pose.pose.position.x = world.first;
    work.pose.pose.position.y = world.second;
    const double goal_yaw = std::atan2(
      region.centroid.second - world.second, region.centroid.first - world.first);
    work.pose.pose.orientation.z = std::sin(goal_yaw / 2.0);
    work.pose.pose.orientation.w = std::cos(goal_yaw / 2.0);
    work.euclid = std::hypot(world.first - rx, world.second - ry);
    const double heading_delta = std::atan2(
      std::sin(goal_yaw - yaw), std::cos(goal_yaw - yaw));
    work.heading = std::abs(heading_delta);
    const auto visible = frontier_exploration_ros2::compute_visible_reveal_gain(
      work.pose.pose, map, costmap, std::nullopt, visible_gain_range_m_,
      visible_gain_fov_deg_, visible_gain_ray_step_deg_, region.visible_reveal_bounds);
    work.gain = visible ? visible->visible_reveal_length_m : 0.0;
    work.map_context = local_context_checksum(
      map, region.centroid.first, region.centroid.second, classification_context_radius_m_);
    work.cost_context = local_context_checksum(
      costmap, world.first, world.second, classification_context_radius_m_);
    work.path_map_context = local_context_checksum(
      map, region.centroid.first, region.centroid.second, path_context_radius_m_);
    work.path_cost_context = local_context_checksum(
      costmap, world.first, world.second, path_context_radius_m_);
    work.last_query_ns = last_query_ns;
    work.cycles_not_queried = cycles_not_queried;
    return work;
  }

  bool cached_context_matches(const EvaluationCache & cache, const Work & work) const
  {
    return cache.geometry_id == work.id && cache.map_context == work.map_context &&
           cache.cost_context == work.cost_context && !cache.classification.empty() &&
           cache.classification != "PLANNER_FAILED";
  }

  bool cached_path_context_matches(const EvaluationCache & cache, const Work & work) const
  {
    return cache.path_map_context == work.path_map_context &&
           cache.path_cost_context == work.path_cost_context && cache.has_work;
  }

  bool work_context_matches_snapshot(
    const Work & work,
    const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & map,
    const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & costmap) const
  {
    if (!map || !costmap) {
      return false;
    }
    frontier_exploration_ros2::OccupancyGrid2d map_grid(map), cost_grid(costmap);
    const auto current_map_context = local_context_checksum(
      map_grid, work.region.centroid.first, work.region.centroid.second,
      classification_context_radius_m_);
    const auto current_cost_context = local_context_checksum(
      cost_grid, work.pose.pose.position.x, work.pose.pose.position.y,
      classification_context_radius_m_);
    const auto current_path_map_context = local_context_checksum(
      map_grid, work.region.centroid.first, work.region.centroid.second,
      path_context_radius_m_);
    const auto current_path_cost_context = local_context_checksum(
      cost_grid, work.pose.pose.position.x, work.pose.pose.position.y,
      path_context_radius_m_);
    return candidate_query_contexts_match(
      work.map_context, current_map_context,
      work.cost_context, current_cost_context,
      work.path_map_context, current_path_map_context,
      work.path_cost_context, current_path_cost_context);
  }

  bool work_context_matches_latest(const Work & work)
  {
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr map, costmap;
    {
      std::lock_guard<std::mutex> lock(mu_);
      map = latest_map_;
      costmap = latest_cost_;
    }
    return work_context_matches_snapshot(work, map, costmap);
  }

  void reject_stale_work(
    const Work & work, uint64_t request_map_revision, uint64_t current_map_revision,
    uint64_t request_costmap_revision, uint64_t current_costmap_revision)
  {
    set_region_status(work.id, "DETECTED_NOT_QUERIED");
    auto & cache = evaluation_cache_[work.id];
    cache.classification = "DETECTED_NOT_QUERIED";
    cache.has_work = false;
    cache.last_query_result = "STALE_REVISION_REJECTED";
    cache.last_query_ns = now().nanoseconds();
    ++cache.cycles_not_queried;
    if (work.tier1_unqueried) {
      ++cycle_tier1_aborted_;
    }
    ++cycle_stale_query_rejections_;
    RCLCPP_INFO(
      get_logger(),
      "FRONTIER_QUERY_LIFECYCLE query_id=%lu id=%lu state=STALE_REVISION_REJECTED "
      "reason=LOCAL_CONTEXT_CHANGED request_map_revision=%lu current_map_revision=%lu "
      "request_costmap_revision=%lu current_costmap_revision=%lu",
      work.query_event_id, work.id, request_map_revision, current_map_revision,
      request_costmap_revision, current_costmap_revision);
  }

  void refresh_publication_context()
  {
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr map, costmap;
    uint64_t map_revision, costmap_revision;
    {
      std::lock_guard<std::mutex> lock(mu_);
      map = latest_map_;
      costmap = latest_cost_;
      map_revision = revision_;
      costmap_revision = cost_revision_;
    }
    if (!map || !costmap) {
      return;
    }
    std::vector<Work> valid;
    valid.reserve(reachable_.size());
    for (const auto & work : reachable_) {
      if (work_context_matches_snapshot(work, map, costmap)) {
        valid.push_back(work);
      } else {
        const auto cache = evaluation_cache_.find(work.id);
        const int64_t accepted_ns = cache == evaluation_cache_.end() ? 0 :
          cache->second.last_query_ns;
        const int64_t current_ns = now().nanoseconds();
        const int64_t accepted_age_ns = current_ns - accepted_ns;
        const bool accepted_recently = accepted_ns > 0 && accepted_age_ns >= 0 &&
          std::chrono::duration<double>(std::chrono::nanoseconds(accepted_age_ns)) <=
          kLocalContextTolerance;
        if (accepted_recently) {
          valid.push_back(work);
        } else {
          reject_stale_work(
            work, cycle_revision_, map_revision, cycle_cost_revision_, costmap_revision);
        }
      }
    }
    reachable_.swap(valid);
    cycle_map_ = map;
    cycle_cost_ = costmap;
    cycle_revision_ = map_revision;
    cycle_cost_revision_ = costmap_revision;
  }

  void set_region_status(uint64_t id, const std::string & status)
  {
    for (auto & diagnostic : region_diagnostics_) {
      if (diagnostic.id == id) {diagnostic.status = status;}
    }
  }

  void count_classification(const std::string & status)
  {
    if (status == "DETECTED_NOT_QUERIED") {++detected_not_queried_count_;}
    else if (status == "SMALL_BELOW_THRESHOLD") {++small_frontier_count_;}
    else if (status == "OUT_OF_RANGE") {++out_of_range_frontier_count_;}
    else if (status == "UNREACHABLE" || status == "UNREACHABLE_SAFE_APPROACH") {
      ++unreachable_frontier_count_;
    } else if (status == "PLANNER_FAILED") {++planner_failure_count_;}
  }

  void recount_region_statuses()
  {
    small_frontier_count_ = 0;
    out_of_range_frontier_count_ = 0;
    unreachable_frontier_count_ = 0;
    planner_failure_count_ = 0;
    detected_not_queried_count_ = 0;
    for (const auto & diagnostic : region_diagnostics_) {
      count_classification(diagnostic.status);
    }
  }

  void start_cycle(
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr map,
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr costmap,
    uint64_t map_revision, uint64_t costmap_revision,
    double rx, double ry, double yaw)
  {
    TimingScope timing(this, TimingSection::START_CYCLE, now().seconds());
    // A normal processing tick may win the race with the delayed retry timer
    // created by a stale-revision abort.  That old timer must not remain
    // attached to the new cycle: its later callback could cancel or suppress
    // the new cycle's query retry timer.
    cancel_retry_timer();
    state_ = State::EXTRACTING;
    ++cycle_sequence_;
    cycle_start_sim_time_ = now().seconds();
    cycle_cache_before_ = evaluation_cache_.size();
    cycle_new_ids_ = 0;
    cycle_reused_ids_ = 0;
    cycle_pruned_absent_ = 0;
    cycle_evicted_capacity_ = 0;
    cycle_identity_associations_ = 0;
    cycle_unique_frontier_records_ = 0;
    cycle_duplicate_id_records_ = 0;
    cycle_tier1_selected_ = 0;
    cycle_tier1_sent_ = 0;
    cycle_tier1_reachable_ = 0;
    cycle_tier1_unreachable_ = 0;
    cycle_tier1_aborted_ = 0;
    cycle_tier1_timeout_ = 0;
    cycle_tier1_remaining_dnu_ = 0;
    cycle_termination_reason_ = "OTHER";
    cycle_old_revision_ = 0;
    cycle_new_revision_ = 0;
    cycle_old_cost_revision_ = 0;
    cycle_new_cost_revision_ = 0;
    cycle_map_cancelled_tier1_ = 0;
    cycle_stale_query_rejections_ = 0;
    const auto started = now();
    cycle_map_ = map;
    cycle_cost_ = costmap;
    cycle_revision_ = map_revision;
    cycle_cost_revision_ = costmap_revision;
    rx_ = rx;
    ry_ = ry;
    yaw_ = yaw;
    works_.clear();
    reachable_.clear();
    region_diagnostics_.clear();
    query_index_ = 0;
    queries_ = 0;
    tier1_queries_issued_ = 0;
    tier2_queries_issued_ = 0;
    tier1_pending_count_ = 0;
    tier2_pending_count_ = 0;
    safe_approach_rejections_ = 0;
    detected_frontier_count_ = 0;
    small_frontier_count_ = 0;
    out_of_range_frontier_count_ = 0;
    unreachable_frontier_count_ = 0;
    planner_failure_count_ = 0;
    detected_not_queried_count_ = 0;
    frontier_exploration_ros2::OccupancyGrid2d grid(map), cost_grid(costmap);
    geometry_msgs::msg::Pose robot_pose;
    robot_pose.position.x = rx;
    robot_pose.position.y = ry;
    robot_pose.orientation.z = std::sin(yaw / 2.0);
    robot_pose.orientation.w = std::cos(yaw / 2.0);
    // Frontier extraction, clustering, reachable-goal generation, visible
    // reveal geometry, map-generation handling, and exact cell retention are
    // provided by the upstream core.  This node only adapts its snapshot into
    // project tasks and performs the final asynchronous path safety evidence.
    if (!core_) {
      state_ = State::IDLE;
      return;
    }
    const auto snapshot_started = std::chrono::steady_clock::now();
    const double snapshot_sim_time_s = now().seconds();
    const auto snapshot = core_->get_frontier_snapshot(robot_pose, minimum_robot_distance_m_);
    const auto & regions = snapshot.frontiers;
    record_timing(
      TimingSection::CORE_FRONTIER_SNAPSHOT, snapshot_sim_time_s,
      std::chrono::duration<double>(
        std::chrono::steady_clock::now() - snapshot_started).count());
    const auto filtering_started = std::chrono::steady_clock::now();
    const double filtering_sim_time_s = now().seconds();
    detected_frontier_count_ = static_cast<uint32_t>(regions.size());
    region_diagnostics_.reserve(regions.size());
    std::vector<FrontierEvaluationRecord> schedule_records;
    std::vector<Work> pending_work;
    std::unordered_set<uint64_t> cycle_frontier_ids;
    cycle_frontier_ids.reserve(regions.size());
    const auto now_ns = now().nanoseconds();

    for (const auto & region : regions) {
      const uint64_t id = stable_frontier_id(region, grid, stable_id_quantization_m_);
      if (!cycle_frontier_ids.insert(id).second) {
        ++cycle_duplicate_id_records_;
        RCLCPP_WARN(
          get_logger(),
          "FRONTIER_ID_DUPLICATE robot=%s id=%lu centroid_x=%.3f centroid_y=%.3f map_revision=%lu",
          robot_id_.c_str(), id, region.centroid.first, region.centroid.second, map_revision);
      }
      if (evaluation_cache_.find(id) == evaluation_cache_.end()) {
        ++cycle_new_ids_;
      } else {
        ++cycle_reused_ids_;
      }
      if (evaluation_cache_.find(id) == evaluation_cache_.end()) {
        const auto current_bounds = frontier_world_bounds(region, grid);
        double best_distance = 0.12;
        const IdentityReference * best = nullptr;
        for (const auto & previous : previous_identity_references_) {
          if (previous.id == id) {continue;}
          const double distance = std::hypot(
            previous.centroid_x - region.centroid.first,
            previous.centroid_y - region.centroid.second);
          const bool overlap_x = previous.bounds[0] <= current_bounds[2] &&
            current_bounds[0] <= previous.bounds[2];
          const bool overlap_y = previous.bounds[1] <= current_bounds[3] &&
            current_bounds[1] <= previous.bounds[3];
          if (distance < best_distance && overlap_x && overlap_y) {
            best_distance = distance;
            best = &previous;
          }
        }
        if (best) {
          ++cycle_identity_associations_;
          RCLCPP_INFO(
            get_logger(),
            "FRONTIER_ID_ASSOCIATION old_id=%lu new_id=%lu centroid_distance_m=%.3f",
            best->id, id, best_distance);
        }
      }
      region_diagnostics_.push_back({id, region, "DETECTED_NOT_QUERIED", 0.0, 0.0, false});
      auto & cache = evaluation_cache_[id];
      cache.geometry_id = id;
      ++cache.cycles_seen;
      cache.last_seen_ns = now_ns;
      const double length = static_cast<double>(region.size) * grid.map().info.resolution;
      if (length + 1e-9 < minimum_frontier_length_m_ || !region.goal_point) {
        set_region_status(id, "SMALL_BELOW_THRESHOLD");
        count_classification("SMALL_BELOW_THRESHOLD");
        continue;
      }
      if (suppressed(id, map_revision)) {
        set_region_status(id, "UNREACHABLE");
        cache.classification = "UNREACHABLE";
        cache.last_query_result = "SUPPRESSED";
        count_classification("UNREACHABLE");
        continue;
      }
      TimingScope make_work_timing(this, TimingSection::MAKE_WORK, now().seconds());
      Work work = make_work(
        region, id, grid, cost_grid, rx, ry, yaw,
        cache.last_query_ns, cache.cycles_not_queried);
      if (!work.pose.header.frame_id.empty()) {
        auto & diagnostic = region_diagnostics_.back();
        diagnostic.approach_x = work.pose.pose.position.x;
        diagnostic.approach_y = work.pose.pose.position.y;
        diagnostic.visible_reveal_gain = work.gain;
        // Nav2's initial path heading is unavailable before querying.  Zero
        // is its safe lower bound; the existing endpoint tolerance is removed
        // from the geometric path lower bound.
        diagnostic.optimistic_cost_lower_bound_s =
          std::max(0.0, work.euclid - goal_tolerance_m_) /
          std::max(cost_only_reference_linear_speed_mps_, 1e-9);
        diagnostic.has_approach = true;
      }
      if (work.pose.header.frame_id.empty()) {
        set_region_status(id, "UNREACHABLE_SAFE_APPROACH");
        cache.classification = require_known_approach_ ?
          "DETECTED_NOT_QUERIED" : "UNREACHABLE";
        cache.last_query_result = require_known_approach_ ?
          "RETRYABLE_NO_KNOWN_SAFE_APPROACH" : "UNREACHABLE_SAFE_APPROACH";
        cache.map_context = local_context_checksum(
          grid, region.centroid.first, region.centroid.second, classification_context_radius_m_);
        cache.cost_context = local_context_checksum(
          cost_grid, region.goal_point->first, region.goal_point->second,
          classification_context_radius_m_);
        cache.path_map_context = 0;
        cache.path_cost_context = 0;
        cache.has_work = false;
        cache.last_query_ns = now_ns;
        cache.query_count++;
        cache.cycles_not_queried = 0;
        ++safe_approach_rejections_;
        count_classification("UNREACHABLE_SAFE_APPROACH");
        if (require_known_approach_) {
          RCLCPP_INFO(
            get_logger(),
            "FRONTIER_APPROACH_RETRYABLE id=%lu reason=NO_KNOWN_FREE_POINT "
            "map_revision=%lu costmap_revision=%lu",
            id, map_revision, costmap_revision);
        } else {
          suppress(id, map_revision, true);
        }
        continue;
      }
      if (!costing_open()) {
        continue;
      }
      const bool never_queried = cache.query_count == 0;
      const bool transient_failure = cache.classification == "PLANNER_FAILED";
      const bool map_context_changed =
        cache.query_count > 0 && cache.map_context != work.map_context;
      const bool cost_context_changed =
        cache.query_count > 0 && cache.cost_context != work.cost_context;
      const bool context_invalidated = map_context_changed || cost_context_changed;
      const bool classification_cached =
        cached_context_matches(cache, work) && cache.classification != "DETECTED_NOT_QUERIED";
      if (classification_cached) {
        set_region_status(id, cache.classification);
        ++classification_cache_hits_;
        RCLCPP_INFO(
          get_logger(), "FRONTIER_CLASSIFICATION_CACHE_HIT id=%lu canonical_id=%016lx status=%s map_revision=%lu costmap_revision=%lu",
          id, id, cache.classification.c_str(), map_revision, costmap_revision);
        if (cache.classification == "REACHABLE" && !cached_path_context_matches(cache, work)) {
          // The high-level classification remains valid, but the exact path
          // is not reused after its stricter local corridor context changes.
          // A fresh query refreshes the bid path; final dispatch validation is
          // still independent and remains mandatory.
          ++path_cache_invalidations_;
          work.path_refresh = true;
          work.cycles_not_queried = 0;
          pending_work.push_back(std::move(work));
          schedule_records.push_back({
              id, false, true, false, 0, cache.last_query_ns,
              std::max(0.0, pending_work.back().euclid - goal_tolerance_m_) /
              std::max(cost_only_reference_linear_speed_mps_, 1e-9), false});
          RCLCPP_INFO(
            get_logger(),
            "FRONTIER_PATH_CACHE_INVALIDATED id=%lu reason=%s map_revision=%lu costmap_revision=%lu",
            id, map_context_changed && cost_context_changed ? "MAP_AND_COSTMAP_CONTEXT" :
            (map_context_changed ? "MAP_CONTEXT" : "COSTMAP_CONTEXT"), map_revision,
            costmap_revision);
          continue;
        }
        if (cache.classification == "REACHABLE" && cache.has_work) {
          reachable_.push_back(cache.work);
        } else {
          count_classification(cache.classification);
        }
        RCLCPP_INFO(
          get_logger(),
          "FRONTIER_QUERY_LIFECYCLE query_id=0 id=%lu state=CACHE_SATISFIED status=%s",
          id, cache.classification.c_str());
        cache.cycles_not_queried = 0;
        continue;
      }
      const std::string old_classification = cache.classification;
      ++classification_cache_misses_;
      if (never_queried) {
        RCLCPP_INFO(
          get_logger(),
          "FRONTIER_CLASSIFICATION_INVALIDATED id=%lu old_status=%s reason=NEVER_QUERIED map_revision=%lu costmap_revision=%lu",
          id, old_classification.empty() ? "NONE" : old_classification.c_str(), map_revision,
          costmap_revision);
      } else {
        const char * reason = transient_failure ? "PREVIOUS_TRANSIENT_FAILURE" :
          (map_context_changed && cost_context_changed ? "MAP_AND_COSTMAP_CONTEXT" :
          (map_context_changed ? "MAP_CONTEXT" :
          (cost_context_changed ? "COSTMAP_CONTEXT" : "OTHER")));
        RCLCPP_INFO(
          get_logger(),
          "FRONTIER_CLASSIFICATION_INVALIDATED id=%lu old_status=%s reason=%s map_changed=%s costmap_changed=%s map_revision=%lu costmap_revision=%lu",
          id, old_classification.empty() ? "NONE" : old_classification.c_str(), reason,
          map_context_changed ? "true" : "false", cost_context_changed ? "true" : "false",
          map_revision, costmap_revision);
      }
      cache.classification = "DETECTED_NOT_QUERIED";
      ++cache.cycles_not_queried;
      work.cycles_not_queried = cache.cycles_not_queried;
      work.last_query_ns = cache.last_query_ns;
      work.tier1_unqueried = true;
      pending_work.push_back(std::move(work));
      schedule_records.push_back({id, never_queried, context_invalidated,
                                  transient_failure, cache.cycles_not_queried,
                                  cache.last_query_ns,
                                  std::max(0.0, pending_work.back().euclid -
                                    goal_tolerance_m_) /
                                  std::max(cost_only_reference_linear_speed_mps_, 1e-9), true});
    }
    cycle_unique_frontier_records_ = cycle_frontier_ids.size();

    record_timing(
      TimingSection::FRONTIER_FILTERING, filtering_sim_time_s,
      std::chrono::duration<double>(
        std::chrono::steady_clock::now() - filtering_started).count());

    if (!costing_open()) {
      cycle_termination_reason_ = "COSTING_PAUSED";
      publish_batch(true);
      state_ = State::IDLE;
      works_.clear();
      reachable_.clear();
      cycle_map_.reset();
      cycle_cost_.reset();
      return;
    }

    normalize_coarse(pending_work);
    tier1_pending_count_ = static_cast<std::size_t>(std::count_if(
      schedule_records.begin(), schedule_records.end(),
      [](const auto & record) {return record.tier1_unqueried;}));
    cycle_tier1_pending_start_ = tier1_pending_count_;
    tier2_pending_count_ = schedule_records.size() - tier1_pending_count_;
    const auto selected_indices = fair_frontier_query_order(
      schedule_records, static_cast<std::size_t>(maximum_candidates_before_path_check_),
      static_cast<std::size_t>(maximum_path_queries_per_cycle_));
    for (const auto index : selected_indices) {
      works_.push_back(std::move(pending_work[index]));
      works_.back().query_event_id = ++query_event_sequence_;
      set_region_status(
        works_.back().id, works_.back().path_refresh ?
        evaluation_cache_[works_.back().id].classification : "DETECTED_NOT_QUERIED");
      RCLCPP_INFO(
        get_logger(), "FRONTIER_QUERY_SELECTED query_id=%lu id=%lu canonical_id=%016lx cycles_not_queried=%lu query_index=%zu",
        works_.back().query_event_id, works_.back().id, works_.back().id,
        works_.back().cycles_not_queried, works_.size() - 1);
      if (works_.back().tier1_unqueried) {++cycle_tier1_selected_;}
    }
    extract_ms_ = (now() - started).seconds() * 1000.0;
    state_ = State::PATH_CHECKING;
    send_next();
  }

  void normalize_coarse(std::vector<Work> & work_items)
  {
    if (work_items.empty()) {return;}
    if (selection_policy_ == "frontier_cost_only") {
      // The bounded pre-query scheduler is deliberately policy-neutral.  A
      // real Nav2 path length is unavailable until ComputePathToPose returns,
      // so no provisional normalized path/heading tradeoff is allowed here.
      // fair_frontier_query_order() uses only bounded freshness/starvation
      // records, not Work::score.
      for (auto & work : work_items) {work.score = 0.0;}
      return;
    }
    auto range = [](const auto & values, auto getter) {
        const auto result = std::minmax_element(
          values.begin(), values.end(), [&getter](const auto & first, const auto & second) {
            return getter(first) < getter(second);
          });
        return std::pair{getter(*result.first), getter(*result.second)};
      };
    const auto gain = range(work_items, [](const Work & work) {return work.gain;});
    const auto distance = range(work_items, [](const Work & work) {return work.euclid;});
    const auto heading = range(work_items, [](const Work & work) {return work.heading;});
    for (auto & work : work_items) {
      work.score = gain_weight_ * normalized_value(work.gain, gain.first, gain.second) -
        distance_weight_ * normalized_value(work.euclid, distance.first, distance.second) -
        heading_weight_ * normalized_value(work.heading, heading.first, heading.second);
    }
  }

  void send_next()
  {
    log_query_boundary("SEND_NEXT_ENTER");
    QueryBoundaryScope boundary(this, "SEND_NEXT_EXIT");
    TimingScope timing(this, TimingSection::QUERY_RESULT_PROCESSING, now().seconds());
    if (!costing_open()) {
      RCLCPP_INFO(
        get_logger(),
        "FRONTIER_QUERY_SUPPRESSED reason=COSTING_CLOSED epoch_id=%lu sim_time=%.6f",
        costing_epoch_id_, now().seconds());
      return;
    }
    if (queries_ >= static_cast<std::size_t>(maximum_path_queries_per_cycle_)) {
      cycle_termination_reason_ = "QUERY_LIMIT_REACHED";
      finish();
      return;
    }
    if (query_index_ >= works_.size()) {
      cycle_termination_reason_ = "WORK_EXHAUSTED";
      finish();
      return;
    }
    if (fallback_priority_requested()) {
      cycle_termination_reason_ = "FALLBACK_PRIORITY_YIELD";
      ++path_priority_yields_;
      schedule_query_retry(10ms);
      return;
    }
    if (!planner_->action_server_is_ready()) {
      cycle_termination_reason_ = "PLANNER_UNAVAILABLE";
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "planner action unavailable");
      for (std::size_t i = query_index_; i < works_.size(); ++i) {
        if (works_[i].tier1_unqueried) {++cycle_tier1_aborted_;}
        set_region_status(works_[i].id, "PLANNER_FAILED");
        evaluation_cache_[works_[i].id].classification = "PLANNER_FAILED";
        evaluation_cache_[works_[i].id].last_query_result =
          "ACTION_SERVER_UNAVAILABLE";
        RCLCPP_INFO(
          get_logger(),
          "FRONTIER_QUERY_LIFECYCLE query_id=%lu id=%lu state=RESULT_RECEIVED status=PLANNER_FAILED reason=ACTION_SERVER_UNAVAILABLE failure_class=ACTION_SERVER_UNAVAILABLE target_x=%.3f target_y=%.3f",
          works_[i].query_event_id, works_[i].id,
          works_[i].pose.pose.position.x, works_[i].pose.pose.position.y);
        RCLCPP_INFO(
          get_logger(),
          "FRONTIER_QUERY_RESULT query_id=%lu id=%lu canonical_id=%016lx status=PLANNER_FAILED action_result=UNKNOWN failure_class=ACTION_SERVER_UNAVAILABLE target_x=%.3f target_y=%.3f error_code=-1 error_name=ACTION_SERVER_UNAVAILABLE duration_s=0.000",
          works_[i].query_event_id, works_[i].id, works_[i].id,
          works_[i].pose.pose.position.x, works_[i].pose.pose.position.y);
      }
      finish();
      return;
    }
    if (!acquire_path_lock()) {
      cycle_termination_reason_ = "PATH_LOCK_RETRY";
      schedule_query_retry(10ms);
      return;
    }
    const auto candidate = works_[query_index_++];
    const auto revision = cycle_revision_;
    const auto cost_revision = cycle_cost_revision_;
    const auto candidate_generation = candidate_generation_id_ + 1;
    const auto request = ++request_generation_;
    // The request owns the query lifecycle from submission onward.  This is
    // deliberately set before async_send_goal() so the watchdog can recover
    // even when Nav2 never delivers a goal response.
    active_request_ = request;
    path_lock_request_ = request;
    ++queries_;
    if (candidate.tier1_unqueried) {
      ++tier1_queries_issued_;
      ++cycle_tier1_sent_;
    } else {
      ++tier2_queries_issued_;
    }
    auto & cache = evaluation_cache_[candidate.id];
    cache.last_query_ns = now().nanoseconds();
    cache.map_context = candidate.map_context;
    cache.cost_context = candidate.cost_context;
    cache.path_map_context = candidate.path_map_context;
    cache.path_cost_context = candidate.path_cost_context;
    cache.cycles_not_queried = 0;
    RCLCPP_INFO(
      get_logger(), "FRONTIER_QUERY_LIFECYCLE query_id=%lu id=%lu state=REQUEST_SENT request_id=%lu candidate_generation_id=%lu map_revision=%lu costmap_revision=%lu map_stamp_s=%.9f costmap_stamp_s=%.9f target_x=%.9f target_y=%.9f target_yaw=%.9f goal_frame=%s",
      candidate.query_event_id, candidate.id, request, candidate_generation,
      revision, cost_revision, stamp_seconds(cycle_map_->header.stamp),
      stamp_seconds(cycle_cost_->header.stamp), candidate.pose.pose.position.x,
      candidate.pose.pose.position.y, orientation_yaw(candidate.pose.pose.orientation),
      candidate.pose.header.frame_id.c_str());
    RCLCPP_INFO(
      get_logger(), "FRONTIER_QUERY_RESULT_PENDING query_id=%lu id=%lu canonical_id=%016lx candidate_generation_id=%lu map_revision=%lu costmap_revision=%lu map_stamp_s=%.9f costmap_stamp_s=%.9f target_x=%.9f target_y=%.9f target_yaw=%.9f goal_frame=%s",
      candidate.query_event_id, candidate.id, candidate.id, candidate_generation,
      revision, cost_revision, stamp_seconds(cycle_map_->header.stamp),
      stamp_seconds(cycle_cost_->header.stamp), candidate.pose.pose.position.x,
      candidate.pose.pose.position.y, orientation_yaw(candidate.pose.pose.orientation),
      candidate.pose.header.frame_id.c_str());
    last_alternative_request_time_s_ = now().seconds();
    RCLCPP_INFO(
      get_logger(),
      "FRONTIER_ALTERNATIVE_PATH_REQUEST request_id=%lu frontier_id=%lu epoch_id=%lu "
      "sim_time=%.6f coordinator_state=%s",
      request, candidate.id, costing_epoch_id_, last_alternative_request_time_s_,
      coordinator_state_name(last_coordinator_state_));
    const auto request_started = std::chrono::steady_clock::now();
    Action::Goal goal;
    goal.goal = candidate.pose;
    goal.planner_id = planner_id_;
    goal.use_start = false;
    auto options = rclcpp_action::Client<Action>::SendGoalOptions();
    options.goal_response_callback = [this, candidate, revision, cost_revision, candidate_generation, request, request_started](GoalHandle::SharedPtr handle) {
        if (!async_request_is_current(request, request_generation_, revision, cycle_revision_, state_ == State::PATH_CHECKING)) {
          RCLCPP_INFO(
            get_logger(),
            "FRONTIER_QUERY_LIFECYCLE query_id=%lu id=%lu state=SUPERSEDED phase=GOAL_RESPONSE",
            candidate.query_event_id, candidate.id);
          if (handle) {
            RCLCPP_INFO(
              get_logger(),
              "FRONTIER_QUERY_LIFECYCLE query_id=%lu id=%lu state=QUERY_CANCEL_REQUESTED "
              "reason=SUPERSEDED phase=GOAL_RESPONSE",
              candidate.query_event_id, candidate.id);
            planner_->async_cancel_goal(handle);
          }
          release_path_lock_for(request);
          ++stale_results_;
          return;
        }
        uint64_t current_map, current_costmap;
        {std::lock_guard<std::mutex> lock(mu_);
          current_map = revision_;
          current_costmap = cost_revision_;
        }
        const auto query_age = std::chrono::steady_clock::now() - request_started;
        if (!work_context_matches_latest(candidate) && query_age > kLocalContextTolerance) {
          cycle_termination_reason_ = "MAP_REVISION_CHANGED";
          cycle_old_revision_ = revision;
          cycle_new_revision_ = current_map;
          cycle_old_cost_revision_ = cost_revision;
          cycle_new_cost_revision_ = current_costmap;
          if (handle) {planner_->async_cancel_goal(handle);}
          ++stale_results_;
          cancel_query_timeout_for(request);
          ++request_generation_;
          active_request_ = 0;
          release_path_lock_for(request);
          reject_stale_work(candidate, revision, current_map, cost_revision, current_costmap);
          send_next();
          return;
        }
        if (!handle) {
          if (candidate.tier1_unqueried) {++cycle_tier1_aborted_;}
          set_region_status(candidate.id, "PLANNER_FAILED");
          auto & cache = evaluation_cache_[candidate.id];
          cache.classification = "PLANNER_FAILED";
          cache.last_query_result = "TRANSIENT_PLANNER_QUERY_FAILURE";
          cache.has_work = false;
          ++cache.query_count;
          ++planner_failure_count_;
          RCLCPP_INFO(
            get_logger(),
            "FRONTIER_QUERY_LIFECYCLE query_id=%lu id=%lu state=RESULT_RECEIVED status=PLANNER_FAILED reason=GOAL_REJECTED failure_class=TRANSIENT_PLANNER_QUERY_FAILURE candidate_generation_id=%lu target_x=%.9f target_y=%.9f target_yaw=%.9f goal_frame=%s",
            candidate.query_event_id, candidate.id, candidate_generation,
            candidate.pose.pose.position.x, candidate.pose.pose.position.y,
            orientation_yaw(candidate.pose.pose.orientation), candidate.pose.header.frame_id.c_str());
          RCLCPP_INFO(
            get_logger(),
            "FRONTIER_QUERY_RESULT query_id=%lu id=%lu canonical_id=%016lx status=PLANNER_FAILED action_result=UNKNOWN failure_class=TRANSIENT_PLANNER_QUERY_FAILURE candidate_generation_id=%lu target_x=%.9f target_y=%.9f target_yaw=%.9f goal_frame=%s error_code=-1 error_name=GOAL_REJECTED duration_s=%.3f",
            candidate.query_event_id, candidate.id, candidate.id, candidate_generation,
            candidate.pose.pose.position.x, candidate.pose.pose.position.y,
            orientation_yaw(candidate.pose.pose.orientation), candidate.pose.header.frame_id.c_str(),
            std::chrono::duration<double>(std::chrono::steady_clock::now() - request_started).count());
          cancel_query_timeout_for(request);
          ++request_generation_;
          active_request_ = 0;
          release_path_lock_for(request);
          schedule_query_retry(1ms);
          return;
        }
        active_ = handle;
        RCLCPP_INFO(
          get_logger(),
          "FRONTIER_QUERY_LIFECYCLE query_id=%lu id=%lu state=GOAL_ACCEPTED candidate_generation_id=%lu",
          candidate.query_event_id, candidate.id, candidate_generation);
      };
    options.result_callback = [this, candidate, revision, cost_revision, candidate_generation, request, request_started](const GoalHandle::WrappedResult & result) {
        TimingScope timing(this, TimingSection::QUERY_RESULT_PROCESSING, now().seconds());
        if (!async_request_is_current(request, active_request_, revision, cycle_revision_, state_ == State::PATH_CHECKING)) {
          RCLCPP_INFO(
            get_logger(),
            "FRONTIER_QUERY_LIFECYCLE query_id=%lu id=%lu state=SUPERSEDED phase=RESULT",
            candidate.query_event_id, candidate.id);
          release_path_lock_for(request);
          ++stale_results_;
          return;
        }
        cancel_query_timeout_for(request);
        active_.reset();
        active_request_ = 0;
        uint64_t current_map, current_costmap;
        {std::lock_guard<std::mutex> lock(mu_);
          current_map = revision_;
          current_costmap = cost_revision_;
        }
        const auto query_age = std::chrono::steady_clock::now() - request_started;
        if (!work_context_matches_latest(candidate) && query_age > kLocalContextTolerance) {
          cycle_termination_reason_ = "MAP_REVISION_CHANGED";
          cycle_old_revision_ = revision;
          cycle_new_revision_ = current_map;
          cycle_old_cost_revision_ = cost_revision;
          cycle_new_cost_revision_ = current_costmap;
          ++stale_results_;
          release_path_lock_for(request);
          reject_stale_work(candidate, revision, current_map, cost_revision, current_costmap);
          send_next();
          return;
        }
        const bool ok = result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result &&
          result.result->error_code == Action::Result::NONE;
        const int error_code = result.result ? result.result->error_code : -1;
        const bool hard = result.result && (
          error_code == Action::Result::START_OUTSIDE_MAP ||
          error_code == Action::Result::GOAL_OCCUPIED ||
          error_code == Action::Result::GOAL_OUTSIDE_MAP ||
          error_code == Action::Result::START_OCCUPIED ||
          error_code == Action::Result::NO_VALID_PATH);
        auto & cache = evaluation_cache_[candidate.id];
        ++cache.query_count;
        const auto duration = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - request_started).count();
        auto length = ok ? extract_nav2_path_cost(result.result->path) : std::nullopt;
        Work report_candidate = candidate;
        bool stepback_rejected = false;
        if (length) {
          auto reachable = candidate;
          nav_msgs::msg::Path effective_path = result.result->path;
          double effective_length = *length;
          if (frontier_goal_stepback_m_ > 1e-9) {
            const auto stepped = step_back_path_pose(
              result.result->path, frontier_goal_stepback_m_);
            if (!stepped) {
              stepback_rejected = true;
              const double planner_length = *length;
              cache.has_work = false;
              cache.map_context = candidate.map_context;
              cache.cost_context = candidate.cost_context;
              cache.path_map_context = candidate.path_map_context;
              cache.path_cost_context = candidate.path_cost_context;
              cache.last_query_ns = now().nanoseconds();
              cache.cycles_not_queried = 0;
              cache.classification = "UNREACHABLE_SAFE_APPROACH";
              cache.last_query_result = "PATH_SHORTER_THAN_STEPBACK";
              length = std::nullopt;
              set_region_status(candidate.id, "UNREACHABLE_SAFE_APPROACH");
              if (candidate.tier1_unqueried) {++cycle_tier1_unreachable_;}
              count_classification("UNREACHABLE_SAFE_APPROACH");
              RCLCPP_INFO(
                get_logger(),
                "FRONTIER_STEPBACK_REJECTED id=%lu reason=PATH_SHORTER_THAN_STEPBACK "
                "planner_path_length_m=%.9f stepback_m=%.9f",
                candidate.id, planner_length, frontier_goal_stepback_m_);
            } else {
              effective_path = stepped->path;
              effective_length = stepped->length;
              reachable.pose = effective_path.poses.back();
              reachable.pose.header.frame_id = global_frame_;
              reachable.pose.header.stamp = now();
              const double goal_yaw = std::atan2(
                candidate.region.centroid.second - reachable.pose.pose.position.y,
                candidate.region.centroid.first - reachable.pose.pose.position.x);
              reachable.pose.pose.orientation.x = 0.0;
              reachable.pose.pose.orientation.y = 0.0;
              reachable.pose.pose.orientation.z = std::sin(goal_yaw / 2.0);
              reachable.pose.pose.orientation.w = std::cos(goal_yaw / 2.0);
              reachable.euclid = std::hypot(
                reachable.pose.pose.position.x - rx_, reachable.pose.pose.position.y - ry_);
              if (cycle_map_ && cycle_cost_) {
                frontier_exploration_ros2::OccupancyGrid2d map_grid(cycle_map_);
                frontier_exploration_ros2::OccupancyGrid2d cost_grid(cycle_cost_);
                const auto visible = frontier_exploration_ros2::compute_visible_reveal_gain(
                  reachable.pose.pose, map_grid, cost_grid, std::nullopt,
                  visible_gain_range_m_, visible_gain_fov_deg_, visible_gain_ray_step_deg_,
                  candidate.region.visible_reveal_bounds);
                reachable.gain = visible ? visible->visible_reveal_length_m : 0.0;
                reachable.map_context = local_context_checksum(
                  map_grid, candidate.region.centroid.first, candidate.region.centroid.second,
                  classification_context_radius_m_);
                reachable.cost_context = local_context_checksum(
                  cost_grid, reachable.pose.pose.position.x, reachable.pose.pose.position.y,
                  classification_context_radius_m_);
                reachable.path_map_context = local_context_checksum(
                  map_grid, candidate.region.centroid.first, candidate.region.centroid.second,
                  path_context_radius_m_);
                reachable.path_cost_context = local_context_checksum(
                  cost_grid, reachable.pose.pose.position.x, reachable.pose.pose.position.y,
                  path_context_radius_m_);
              }
            }
          }
          if (!stepback_rejected) {
            reachable.path = effective_length;
            if (selection_policy_ == "frontier_cost_only") {
              // Cost-only heading is the initial direction of the actual valid
              // Nav2 path, measured against the robot heading at query time.
              reachable.heading = path_initial_heading_cost(
                effective_path, yaw_).value_or(0.0);
            } else if (frontier_goal_stepback_m_ > 1e-9) {
              const double final_yaw = orientation_yaw(reachable.pose.pose.orientation);
              reachable.heading = std::abs(std::atan2(
                std::sin(final_yaw - yaw_), std::cos(final_yaw - yaw_)));
            }
          }
          const auto & poses = effective_path.poses;
          const std::size_t count = std::min<std::size_t>(32, poses.size());
          reachable.path_samples.clear();
          reachable.path_samples.reserve(count);
          for (std::size_t i = 0; i < count; ++i) {
            const auto index = count < 2 ? 0 : std::llround(
              static_cast<double>(i) * (poses.size() - 1) / (count - 1));
            geometry_msgs::msg::Point point;
            point.x = poses[index].pose.position.x;
            point.y = poses[index].pose.position.y;
            reachable.path_samples.push_back(point);
          }
          if (!stepback_rejected) {
            if (candidate.tier1_unqueried) {++cycle_tier1_reachable_;}
            const auto diagnostic_length = path_length(
              effective_path, rx_, ry_, reachable.pose.pose.position.x,
              reachable.pose.pose.position.y, goal_tolerance_m_);
            log_path_validation(
              reachable, result, diagnostic_length, candidate_generation, duration,
              &effective_path);
            cache.work = reachable;
            cache.has_work = true;
            cache.map_context = reachable.map_context;
            cache.cost_context = reachable.cost_context;
            cache.path_map_context = reachable.path_map_context;
            cache.path_cost_context = reachable.path_cost_context;
            cache.last_query_ns = now().nanoseconds();
            cache.cycles_not_queried = 0;
            // A successful finite Nav2 path is reachable unless the requested
            // physical step-back cannot be represented on that same path.
            cache.classification = "REACHABLE";
            set_region_status(candidate.id, "REACHABLE");
            for (auto & diagnostic : region_diagnostics_) {
              if (diagnostic.id == candidate.id) {
                diagnostic.approach_x = reachable.pose.pose.position.x;
                diagnostic.approach_y = reachable.pose.pose.position.y;
                diagnostic.visible_reveal_gain = reachable.gain;
                diagnostic.optimistic_cost_lower_bound_s =
                  std::max(0.0, reachable.euclid - goal_tolerance_m_) /
                  std::max(cost_only_reference_linear_speed_mps_, 1e-9);
                diagnostic.has_approach = true;
              }
            }
            report_candidate = reachable;
            length = effective_length;
            reachable_.push_back(std::move(reachable));
          }
        } else {
          if (candidate.tier1_unqueried) {
            if (hard) {++cycle_tier1_unreachable_;} else {++cycle_tier1_aborted_;}
          }
          cache.has_work = false;
          cache.map_context = candidate.map_context;
          cache.cost_context = candidate.cost_context;
          cache.path_map_context = candidate.path_map_context;
          cache.path_cost_context = candidate.path_cost_context;
          cache.last_query_ns = now().nanoseconds();
          cache.cycles_not_queried = 0;
          if (hard) {
            cache.classification = "UNREACHABLE";
            set_region_status(candidate.id, "UNREACHABLE");
            count_classification("UNREACHABLE");
            suppress(candidate.id, revision, false);
          } else {
            cache.classification = "PLANNER_FAILED";
            set_region_status(candidate.id, "PLANNER_FAILED");
            ++planner_failure_count_;
          }
        }
        if (stepback_rejected) {
          report_candidate = candidate;
          cache.last_query_result = "PATH_SHORTER_THAN_STEPBACK";
        }
        cache.last_query_result = cache.classification;
        const auto result_failure_class = [&]() {
            if (stepback_rejected) {return "UNREACHABLE_SAFE_APPROACH";}
            return ok ? "PATH_SUCCESS" : (hard ? "CANDIDATE_UNREACHABLE" :
              query_failure_class(result.code, error_code));
          }();
        RCLCPP_INFO(
          get_logger(), "FRONTIER_QUERY_RESULT query_id=%lu id=%lu canonical_id=%016lx status=%s action_result=%s failure_class=%s candidate_generation_id=%lu target_x=%.9f target_y=%.9f target_yaw=%.9f goal_frame=%s error_code=%d error_name=%s error_message=\"%s\" duration_s=%.3f path_length_m=%.6f",
          report_candidate.query_event_id, report_candidate.id, report_candidate.id,
          cache.classification.c_str(),
          action_result_name(result.code),
          result_failure_class,
          candidate_generation, report_candidate.pose.pose.position.x,
          report_candidate.pose.pose.position.y,
          orientation_yaw(report_candidate.pose.pose.orientation),
          report_candidate.pose.header.frame_id.c_str(),
          error_code, nav2_error_name(error_code),
          result.result ? log_safe(result.result->error_msg).c_str() : "NO_RESULT",
          duration, length.value_or(-1.0));
        RCLCPP_INFO(
          get_logger(),
          "FRONTIER_QUERY_LIFECYCLE query_id=%lu id=%lu state=RESULT_RECEIVED status=%s action_result=%s failure_class=%s candidate_generation_id=%lu error_code=%d error_name=%s duration_s=%.3f",
          candidate.query_event_id, candidate.id, cache.classification.c_str(),
          action_result_name(result.code),
          ok ? "PATH_SUCCESS" : (hard ? "CANDIDATE_UNREACHABLE" :
          query_failure_class(result.code, error_code)), candidate_generation,
          error_code, nav2_error_name(error_code), duration);
        release_path_lock();
        schedule_query_retry(1ms);
      };
    // The deadline belongs to the submitted request, not only to an accepted
    // Nav2 goal.  If goal response delivery is lost, the response callback
    // cannot be the place that starts the watchdog: this request would then
    // retain active_request_ and the path lock indefinitely.
    auto query_timeout_timer = create_wall_timer(
      std::chrono::duration<double>(path_query_timeout_s_),
      [this, candidate, revision, cost_revision, candidate_generation, request, request_started] {
        const auto current_active_request = active_request_.load();
        const auto current_request_generation = request_generation_.load();
        const auto timer_owner_request = timeout_timer_request_.load();
        RCLCPP_INFO(
          get_logger(),
          "FRONTIER_QUERY_WATCHDOG_CALLBACK_ENTERED query_id=%lu id=%lu request_id=%lu "
          "captured_generation=%lu current_active_request=%lu "
          "current_generation=%lu timer_owner_request=%lu state=%d",
          candidate.query_event_id, candidate.id, request, candidate_generation,
          current_active_request, current_request_generation, timer_owner_request,
          static_cast<int>(state_.load()));
        if (request != active_request_ || request != request_generation_ ||
          state_ != State::PATH_CHECKING)
        {
          const char * reason = request != current_active_request ? "ACTIVE_REQUEST_MISMATCH" :
            request != current_request_generation ? "REQUEST_GENERATION_MISMATCH" :
            "STATE_NOT_PATH_CHECKING";
          RCLCPP_INFO(
            get_logger(),
            "FRONTIER_QUERY_WATCHDOG_EARLY_RETURN query_id=%lu id=%lu request_id=%lu "
            "reason=%s current_active_request=%lu current_generation=%lu "
            "timer_owner_request=%lu state=%d",
            candidate.query_event_id, candidate.id, request, reason,
            current_active_request, current_request_generation, timer_owner_request,
            static_cast<int>(state_.load()));
          return;
        }
        if (timer_owner_request != request) {
          RCLCPP_INFO(
            get_logger(),
            "FRONTIER_QUERY_WATCHDOG_OWNER_MISMATCH query_id=%lu id=%lu request_id=%lu "
            "timer_owner_request=%lu",
            candidate.query_event_id, candidate.id, request, timer_owner_request);
        }
        cycle_termination_reason_ = "QUERY_TIMEOUT";
        if (candidate.tier1_unqueried) {++cycle_tier1_timeout_;}
        if (active_) {
          RCLCPP_INFO(
            get_logger(),
            "FRONTIER_QUERY_LIFECYCLE query_id=%lu id=%lu state=QUERY_CANCEL_REQUESTED "
            "candidate_generation_id=%lu",
            candidate.query_event_id, candidate.id, candidate_generation);
          planner_->async_cancel_goal(active_);
          active_.reset();
        }
        // Invalidate both the goal-response and result callbacks.  A late
        // callback from this request must not release or mutate a newer one.
        ++request_generation_;
        active_request_ = 0;
        set_region_status(candidate.id, "PLANNER_FAILED");
        auto & cache = evaluation_cache_[candidate.id];
        cache.classification = "PLANNER_FAILED";
        cache.last_query_result = "PLANNER_QUERY_TIMEOUT";
        cache.has_work = false;
        ++cache.query_count;
        ++planner_failure_count_;
        RCLCPP_INFO(
          get_logger(),
          "FRONTIER_QUERY_LIFECYCLE query_id=%lu id=%lu state=QUERY_TIMEOUT "
          "status=PLANNER_FAILED reason=TIMEOUT failure_class=PLANNER_QUERY_TIMEOUT "
          "candidate_generation_id=%lu request_map_revision=%lu "
          "request_costmap_revision=%lu target_x=%.9f target_y=%.9f target_yaw=%.9f "
          "goal_frame=%s",
          candidate.query_event_id, candidate.id, candidate_generation,
          revision, cost_revision, candidate.pose.pose.position.x,
          candidate.pose.pose.position.y, orientation_yaw(candidate.pose.pose.orientation),
          candidate.pose.header.frame_id.c_str());
        RCLCPP_INFO(
          get_logger(),
          "FRONTIER_QUERY_RESULT query_id=%lu id=%lu canonical_id=%016lx "
          "status=PLANNER_FAILED action_result=TIMEOUT failure_class=PLANNER_QUERY_TIMEOUT "
          "candidate_generation_id=%lu error_code=%d error_name=TIMEOUT duration_s=%.3f",
          candidate.query_event_id, candidate.id, candidate.id, candidate_generation,
          Action::Result::TIMEOUT,
          std::chrono::duration<double>(std::chrono::steady_clock::now() - request_started).count());
        cancel_query_timeout_owned_by(request);
        release_path_lock_for(request);
        schedule_query_retry(1ms);
      });
    log_query_boundary(
      "TIMEOUT_TIMER_MUTEX_BEFORE_LOCK", request, candidate_generation, query_index_, queries_);
    {
      std::lock_guard<std::mutex> lock(timeout_timer_mu_);
      timeout_timer_ = std::move(query_timeout_timer);
      timeout_timer_request_ = request;
    }
    log_query_boundary(
      "TIMEOUT_TIMER_MUTEX_AFTER_UNLOCK", request, candidate_generation, query_index_, queries_);
    RCLCPP_INFO(
      get_logger(),
      "FRONTIER_QUERY_LIFECYCLE query_id=%lu id=%lu state=WATCHDOG_ARMED "
      "candidate_generation_id=%lu request_id=%lu timeout_s=%.3f",
      candidate.query_event_id, candidate.id, candidate_generation, request,
      path_query_timeout_s_);
    log_query_boundary(
      "ASYNC_SEND_GOAL_BEFORE", request, candidate_generation, query_index_, queries_);
    auto goal_future = planner_->async_send_goal(goal, options);
    log_query_boundary(
      "ASYNC_SEND_GOAL_AFTER", request, candidate_generation, query_index_, queries_);
    (void)goal_future;
    log_query_boundary(
      "ASYNC_SEND_GOAL_FUTURE_RECEIVED", request, candidate_generation, query_index_, queries_);
  }

  void finish(bool publish = true)
  {
    for (std::size_t i = query_index_; i < works_.size(); ++i) {
      RCLCPP_INFO(
        get_logger(),
        "FRONTIER_QUERY_LIFECYCLE query_id=%lu id=%lu state=CANCELLED_BEFORE_REQUEST",
        works_[i].query_event_id, works_[i].id);
    }
    request_generation_++;
    active_request_ = 0;
    cancel_query_timeout();
    cancel_retry_timer();
    release_path_lock();
    state_ = State::PUBLISHING;
    bool retry_latest_cycle = false;
    if (publish) {
      refresh_publication_context();
      retry_latest_cycle = cycle_stale_query_rejections_ > 0 && reachable_.empty();
      if (retry_latest_cycle) {
        cycle_termination_reason_ = "STALE_REVISION_NO_VALID_CANDIDATES";
      }
    }
    if (publish && !retry_latest_cycle) {
      normalize_final();
      publish_batch();
      cycle_tier1_remaining_dnu_ = static_cast<std::size_t>(std::count_if(
        region_diagnostics_.begin(), region_diagnostics_.end(),
        [](const auto & diagnostic) {return diagnostic.status == "DETECTED_NOT_QUERIED";}));
    }
    if (event_driven_costing_) {
      if (costing_epoch_open_.exchange(false)) {
        RCLCPP_INFO(
          get_logger(),
          "FRONTIER_COSTING_EPOCH event=CLOSE epoch_id=%lu sim_time=%.6f reason=%s "
          "last_alternative_request_time=%.6f active_request=%lu",
          costing_epoch_id_, now().seconds(),
          query_termination_name(cycle_termination_reason_),
          last_alternative_request_time_s_, active_request_.load());
      }
      retry_latest_cycle = false;
    }
    cycle_cache_before_ = evaluation_cache_.size();
    prune_evaluation_cache();
    RCLCPP_WARN(
      get_logger(),
      "FRONTIER_LIFECYCLE robot=%s cycle=%lu sim_time=%.3f snapshot_frontiers=%u unique_frontier_records=%zu duplicate_id_records=%zu cache_before=%zu cache_after=%zu reused_ids=%zu new_ids=%zu pruned_absent=%zu evicted_capacity=%zu identity_associations=%zu dnu=%u tier1_pending_start=%zu tier1_selected=%zu tier1_sent=%zu tier1_reachable=%zu tier1_unreachable=%zu tier1_aborted=%zu tier1_timeout=%zu tier1_remaining_dnu=%zu termination=%s old_revision=%lu new_revision=%lu old_costmap_revision=%lu new_costmap_revision=%lu map_cancelled_tier1=%zu stale_query_rejections=%zu",
      robot_id_.c_str(), cycle_sequence_, cycle_start_sim_time_, detected_frontier_count_,
      cycle_unique_frontier_records_, cycle_duplicate_id_records_, cycle_cache_before_,
      evaluation_cache_.size(), cycle_reused_ids_, cycle_new_ids_,
      cycle_pruned_absent_, cycle_evicted_capacity_, cycle_identity_associations_,
      detected_not_queried_count_, cycle_tier1_pending_start_, cycle_tier1_selected_,
      cycle_tier1_sent_, cycle_tier1_reachable_, cycle_tier1_unreachable_,
      cycle_tier1_aborted_, cycle_tier1_timeout_, cycle_tier1_remaining_dnu_,
      query_termination_name(cycle_termination_reason_), cycle_old_revision_,
      cycle_new_revision_, cycle_old_cost_revision_, cycle_new_cost_revision_,
      cycle_map_cancelled_tier1_, cycle_stale_query_rejections_);
    state_ = State::IDLE;
    works_.clear();
    reachable_.clear();
    cycle_map_.reset();
    cycle_cost_.reset();
    if (retry_latest_cycle) {
      schedule_latest_cycle_retry();
    }
  }

  void prune_evaluation_cache()
  {
    // Keep only records seen in the most recent upstream snapshot, then apply
    // a hard capacity to bound retained path samples and frontier cells.
    std::unordered_set<uint64_t> current_ids;
    current_ids.reserve(region_diagnostics_.size());
    for (const auto & diagnostic : region_diagnostics_) {
      current_ids.insert(diagnostic.id);
    }
    for (auto it = evaluation_cache_.begin(); it != evaluation_cache_.end();) {
      if (current_ids.find(it->first) == current_ids.end()) {
        ++cycle_pruned_absent_;
        it = evaluation_cache_.erase(it);
      } else {
        ++it;
      }
    }
    while (evaluation_cache_.size() > static_cast<std::size_t>(maximum_evaluation_records_)) {
      auto victim = evaluation_cache_.begin();
      for (auto it = evaluation_cache_.begin(); it != evaluation_cache_.end(); ++it) {
        if (it->second.last_seen_ns < victim->second.last_seen_ns) {
          victim = it;
        }
      }
      ++cycle_evicted_capacity_;
      evaluation_cache_.erase(victim);
    }
    RCLCPP_INFO(
      get_logger(), "FRONTIER_EVALUATION_CACHE_BOUND size=%zu capacity=%d",
      evaluation_cache_.size(), maximum_evaluation_records_);
  }

  bool acquire_path_lock()
  {
    ++path_lock_acquire_attempts_;
    if (path_lock_fd_ >= 0) {return true;}
    path_lock_fd_ = ::open(path_query_lock_path_.c_str(), O_CREAT | O_RDWR, 0666);
    if (path_lock_fd_ < 0 || ::flock(path_lock_fd_, LOCK_EX | LOCK_NB) != 0) {
      if (path_lock_fd_ >= 0) {::close(path_lock_fd_);}
      path_lock_fd_ = -1;
      ++path_lock_retry_count_;
      if (!path_lock_waiting_) {
        path_lock_waiting_ = true;
        path_lock_wait_started_ = std::chrono::steady_clock::now();
      }
      return false;
    }
    if (path_lock_waiting_) {
      const double wait_s = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - path_lock_wait_started_).count();
      path_lock_wait_total_s_ += wait_s;
      path_lock_wait_max_s_ = std::max(path_lock_wait_max_s_, wait_s);
      path_lock_waiting_ = false;
    }
    path_lock_acquired_at_ = std::chrono::steady_clock::now();
    return true;
  }

  void release_path_lock()
  {
    if (path_lock_fd_ >= 0) {
      const double hold_s = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - path_lock_acquired_at_).count();
      ++path_lock_hold_count_;
      path_lock_hold_total_s_ += hold_s;
      path_lock_hold_max_s_ = std::max(path_lock_hold_max_s_, hold_s);
      ::flock(path_lock_fd_, LOCK_UN);
      ::close(path_lock_fd_);
      path_lock_fd_ = -1;
    }
    path_lock_request_ = 0;
  }

  void release_path_lock_for(uint64_t request)
  {
    if (path_lock_fd_ >= 0 && path_lock_request_ == request) {
      release_path_lock();
    }
  }

  void cancel_query_timeout()
  {
    std::lock_guard<std::mutex> lock(timeout_timer_mu_);
    if (timeout_timer_) {
      timeout_timer_->cancel();
      timeout_timer_.reset();
    }
    timeout_timer_request_ = 0;
  }

  void cancel_query_timeout_for(uint64_t request)
  {
    const auto current_active_request = active_request_.load();
    if (request != current_active_request) {
      RCLCPP_INFO(
        get_logger(),
        "FRONTIER_QUERY_WATCHDOG_CLEANUP_SKIPPED request_id=%lu "
        "reason=ACTIVE_REQUEST_MISMATCH current_active_request=%lu "
        "timer_owner_request=%lu",
        request, current_active_request, timeout_timer_request_.load());
      return;
    }
    cancel_query_timeout_owned_by(request);
  }

  void cancel_query_timeout_owned_by(uint64_t request)
  {
    std::lock_guard<std::mutex> lock(timeout_timer_mu_);
    const auto timer_owner_request = timeout_timer_request_.load();
    if (request != timer_owner_request) {
      RCLCPP_INFO(
        get_logger(),
        "FRONTIER_QUERY_WATCHDOG_CLEANUP_SKIPPED request_id=%lu "
        "reason=TIMER_OWNER_MISMATCH timer_owner_request=%lu",
        request, timer_owner_request);
      return;
    }
    if (timeout_timer_) {
      timeout_timer_->cancel();
      timeout_timer_.reset();
    }
    timeout_timer_request_ = 0;
  }

  void log_query_boundary(
    const char * event, uint64_t request = 0, uint64_t generation = 0,
    std::size_t query_index = std::numeric_limits<std::size_t>::max(),
    std::size_t queries = std::numeric_limits<std::size_t>::max())
  {
    const auto effective_query_index = query_index == std::numeric_limits<std::size_t>::max() ?
      query_index_ : query_index;
    const auto effective_queries = queries == std::numeric_limits<std::size_t>::max() ?
      queries_ : queries;
    const auto thread_id = std::hash<std::thread::id>{}(std::this_thread::get_id());
    const auto wall_s = std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
    RCLCPP_INFO(
      get_logger(),
      "FRONTIER_QUERY_BOUNDARY event=%s request_id=%lu generation=%lu query_index=%zu "
      "queries=%zu active_request=%lu request_generation=%lu timer_owner_request=%lu "
      "retry_generation=%lu thread_id=%zu wall_s=%.9f",
      event, request, generation, effective_query_index, effective_queries,
      active_request_.load(), request_generation_.load(), timeout_timer_request_.load(),
      retry_generation_, thread_id, wall_s);
  }

  void schedule_query_retry(std::chrono::milliseconds delay)
  {
    if (retry_timer_) {return;}
    const auto callback_generation = ++retry_generation_;
    retry_timer_ = create_wall_timer(delay, [this, callback_generation] {
      QueryBoundaryScope boundary(this, "RETRY_CALLBACK_EXIT");
      log_query_boundary("RETRY_CALLBACK_ENTER");
      if (!candidate_retry_callback_is_current(callback_generation, retry_generation_)) {
        return;
      }
      if (!costing_open()) {
        RCLCPP_INFO(
          get_logger(),
          "FRONTIER_QUERY_SUPPRESSED reason=RETRY_COSTING_CLOSED epoch_id=%lu sim_time=%.6f",
          costing_epoch_id_, now().seconds());
        return;
      }
      if (retry_timer_) {retry_timer_->cancel();}
      retry_timer_.reset();
      send_next();
    });
  }

  void schedule_latest_cycle_retry()
  {
    if (retry_timer_) {return;}
    const auto delay_ms = static_cast<int64_t>(std::max(
      1.0, 1000.0 / std::max(0.01, processing_rate_hz_)));
    const auto callback_generation = ++retry_generation_;
    retry_timer_ = create_wall_timer(std::chrono::milliseconds(delay_ms), [this, callback_generation] {
      if (!candidate_retry_callback_is_current(callback_generation, retry_generation_)) {
        return;
      }
      if (retry_timer_) {
        retry_timer_->cancel();
        retry_timer_.reset();
      }
      bool has_map, has_costmap;
      {
        std::lock_guard<std::mutex> lock(mu_);
        has_map = static_cast<bool>(latest_map_);
        has_costmap = static_cast<bool>(latest_cost_);
      }
      if (!candidate_cycle_retry_ready(
          processing_active_, state_ == State::IDLE, has_map, has_costmap)) {
        return;
      }
      if (!costing_open()) {
        RCLCPP_INFO(
          get_logger(),
          "FRONTIER_QUERY_SUPPRESSED reason=CYCLE_RETRY_COSTING_CLOSED epoch_id=%lu sim_time=%.6f",
          costing_epoch_id_, now().seconds());
        return;
      }
      tick();
    });
  }

  void cancel_retry_timer()
  {
    ++retry_generation_;
    if (retry_timer_) {
      retry_timer_->cancel();
      retry_timer_.reset();
    }
  }

  bool fallback_priority_requested()
  {
    const int fd = ::open(path_priority_path_.c_str(), O_RDONLY);
    if (fd < 0) {return false;}
    char buffer[64] = {};
    const ssize_t count = ::read(fd, buffer, sizeof(buffer) - 1);
    ::close(fd);
    if (count <= 0) {return false;}
    char * end = nullptr;
    const long owner = std::strtol(buffer, &end, 10);
    if (owner <= 0 || end == buffer) {return false;}
    if (::kill(static_cast<pid_t>(owner), 0) == 0 || errno == EPERM) {
      return true;
    }
    if (errno == ESRCH) {
      ::unlink(path_priority_path_.c_str());
    }
    return false;
  }

  void normalize_final()
  {
    if (reachable_.empty()) {return;}
    double g0 = 1e99, g1 = -1e99, p0 = 1e99, p1 = -1e99;
    double h0 = 1e99, h1 = -1e99;
    for (const auto & work : reachable_) {
      g0 = std::min(g0, work.gain); g1 = std::max(g1, work.gain);
      p0 = std::min(p0, work.path); p1 = std::max(p1, work.path);
      h0 = std::min(h0, work.heading); h1 = std::max(h1, work.heading);
    }
    for (auto & work : reachable_) {
      if (selection_policy_ == "frontier_cost_only") {
        const auto motion_cost = nominal_motion_cost_s(
          work.path, work.heading, cost_only_reference_linear_speed_mps_,
          cost_only_reference_angular_speed_radps_);
        work.score = motion_cost ? -*motion_cost : 0.0;
      } else {
        work.score = gain_weight_ * normalized_value(work.gain, g0, g1) -
          path_weight_ * normalized_value(work.path, p0, p1) -
          heading_weight_ * normalized_value(work.heading, h0, h1);
      }
    }
    std::sort(reachable_.begin(), reachable_.end(), [this](const Work & first, const Work & second) {
      if (first.score != second.score) {return first.score > second.score;}
      if (first.path != second.path) {return first.path < second.path;}
      if (first.heading != second.heading) {return first.heading < second.heading;}
      if (selection_policy_ != "frontier_cost_only" && first.gain != second.gain) {
        return first.gain > second.gain;
      }
      return first.id < second.id;
    });
  }

  void apply_upstream_route_ordering()
  {
    if (!upstream_route_ordering_enabled_ || reachable_.empty() || !core_ ||
        !cycle_map_) {
      return;
    }
    geometry_msgs::msg::Pose pose;
    pose.position.x = rx_;
    pose.position.y = ry_;
    pose.orientation.z = std::sin(yaw_ / 2.0);
    pose.orientation.w = std::cos(yaw_ / 2.0);
    frontier_exploration_ros2::FrontierSequence candidates;
    candidates.reserve(reachable_.size());
    for (const auto & work : reachable_) {
      candidates.push_back(work.region);
    }
    const auto ordered = core_->build_mrtsp_frontier_sequence(candidates, pose);
    frontier_exploration_ros2::OccupancyGrid2d grid(cycle_map_);
    std::unordered_map<uint64_t, uint32_t> ranks;
    for (std::size_t rank = 0; rank < ordered.size(); ++rank) {
      const uint64_t id = stable_frontier_id(
        ordered[rank], grid, stable_id_quantization_m_);
      // The solver selects every frontier at most once.  Retaining the first
      // rank also gives a deterministic answer if two transient IDs happen
      // to quantize together in the adapter identity layer.
      ranks.emplace(id, static_cast<uint32_t>(rank));
    }
    ++mrtsp_route_generation_;
    for (auto & work : reachable_) {
      const auto found = ranks.find(work.id);
      work.mrtsp_route_rank = found == ranks.end() ?
        std::numeric_limits<uint32_t>::max() : found->second;
      work.mrtsp_route_generation = mrtsp_route_generation_;
    }
    std::stable_sort(
      reachable_.begin(), reachable_.end(), [](const Work & first, const Work & second) {
        if (first.mrtsp_route_rank != second.mrtsp_route_rank) {
          return first.mrtsp_route_rank < second.mrtsp_route_rank;
        }
        // This preserves a useful publication order outside the bounded DP
        // horizon.  The scalar remains local diagnostic data, never a team
        // utility in the corrected distributed mode.
        if (first.score != second.score) {return first.score > second.score;}
        return first.id < second.id;
      });
    RCLCPP_INFO(
      get_logger(),
      "UPSTREAM_ROUTE_PLAN robot=%s generation=%lu solver=%s reachable=%zu ordered=%zu",
      robot_id_.c_str(), mrtsp_route_generation_, upstream_mrtsp_solver_.c_str(),
      reachable_.size(), ordered.size());
  }

  std::string diagnostic_regions_json() const
  {
    if (!diagnostic_frontier_capture_ || !cycle_map_) {return {};}
    frontier_exploration_ros2::OccupancyGrid2d grid(cycle_map_);
    const auto & q = cycle_map_->info.origin.orientation;
    const double origin_yaw = std::atan2(
      2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    std::ostringstream output;
    output << std::setprecision(8)
           << "{\"schema\":2,\"robot_id\":\"" << robot_id_
           << "\",\"map_revision\":" << cycle_revision_
           << ",\"costmap_revision\":" << cycle_cost_revision_
           << ",\"candidate_generation_id\":" << (candidate_generation_id_ + 1)
           << ",\"frame\":\"" << global_frame_ << "\",\"resolution\":"
           << cycle_map_->info.resolution << ",\"width\":" << cycle_map_->info.width
           << ",\"height\":" << cycle_map_->info.height << ",\"origin\":["
           << cycle_map_->info.origin.position.x << ","
           << cycle_map_->info.origin.position.y << "," << origin_yaw << "],\"regions\":[";
    for (std::size_t i = 0; i < region_diagnostics_.size(); ++i) {
      const auto & diagnostic = region_diagnostics_[i];
      const auto bounds = frontier_world_bounds(diagnostic.region, grid);
      if (i) {output << ',';}
      output << "{\"id\":" << diagnostic.id
             << ",\"physical_id\":" << diagnostic.id
             << ",\"canonical_id\":\"" << std::hex << std::setw(16)
             << std::setfill('0') << diagnostic.id << std::dec << std::setfill(' ')
             << "\",\"status\":\"" << diagnostic.status
             << "\",\"visible_reveal_gain\":";
      if (std::isfinite(diagnostic.visible_reveal_gain)) {
        output << diagnostic.visible_reveal_gain;
      } else {
        output << "null";
      }
      output
             << ",\"cell_count\":" << diagnostic.region.size
             << ",\"size_m\":" <<
                (diagnostic.region.size * grid.map().info.resolution)
             << ",\"query_count\":" << evaluation_cache_.at(diagnostic.id).query_count
             << ",\"cycles_seen\":" << evaluation_cache_.at(diagnostic.id).cycles_seen
             << ",\"cycles_not_queried\":" << evaluation_cache_.at(diagnostic.id).cycles_not_queried
             << ",\"last_query_ns\":" << evaluation_cache_.at(diagnostic.id).last_query_ns
             << ",\"last_query_result\":\""
             << evaluation_cache_.at(diagnostic.id).last_query_result << "\""
             << ",\"centroid\":[" << diagnostic.region.centroid.first << ","
             << diagnostic.region.centroid.second << "],\"bbox\":[" << bounds[0]
             << "," << bounds[1] << "," << bounds[2] << "," << bounds[3]
             << "],\"approach\":";
      if (diagnostic.has_approach) {
        output << '[' << diagnostic.approach_x << ',' << diagnostic.approach_y << ']';
      } else {
        output << "null";
      }
      output << ",\"cells\":[";
      append_region_cells_json(output, diagnostic.region, 0);
      output << "]}";
    }
    output << "]}";
    return output.str();
  }

  std::string terminal_frontier_regions_json() const
  {
    if (!cycle_map_) {return {};}  // bounded summary for normal production
    std::ostringstream output;
    output << std::setprecision(8) << "{\"resolution\":"
           << cycle_map_->info.resolution
           << ",\"map_revision\":" << cycle_revision_
           << ",\"costmap_revision\":" << cycle_cost_revision_
           << ",\"candidate_generation_id\":" << (candidate_generation_id_ + 1)
           << ",\"regions\":[";
    for (std::size_t i = 0; i < region_diagnostics_.size(); ++i) {
      const auto & diagnostic = region_diagnostics_[i];
      if (i) {output << ',';}
      output << "{\"physical_id\":" << diagnostic.id
             << ",\"status\":\"" << diagnostic.status
             << "\",\"visible_reveal_gain\":";
      if (std::isfinite(diagnostic.visible_reveal_gain)) {
        output << diagnostic.visible_reveal_gain;
      } else {
        output << "null";
      }
      output
             << ",\"cell_count\":" << diagnostic.region.size
             << ",\"size_m\":" <<
                (diagnostic.region.size * cycle_map_->info.resolution)
             << ",\"query_count\":" << evaluation_cache_.at(diagnostic.id).query_count
             << ",\"cycles_seen\":" << evaluation_cache_.at(diagnostic.id).cycles_seen
             << ",\"cycles_not_queried\":" << evaluation_cache_.at(diagnostic.id).cycles_not_queried
             << ",\"last_query_ns\":" << evaluation_cache_.at(diagnostic.id).last_query_ns
             << ",\"last_query_result\":\""
             << evaluation_cache_.at(diagnostic.id).last_query_result << "\""
             << ",\"optimistic_cost_lower_bound_s\":";
      if (std::isfinite(diagnostic.optimistic_cost_lower_bound_s)) {
        output << diagnostic.optimistic_cost_lower_bound_s;
      } else {
        output << "null";
      }
      output << "}";
    }
    output << "]}";
    return output.str();
  }

  std::string lower_bound_context_fingerprint() const
  {
    // The lower bound uses the current robot position and the current
    // candidate-generation context, not map revision alone. Keep the wire
    // value compact while hashing every input that can change the emitted
    // region summary or its optimistic motion bound.
    std::ostringstream payload;
    payload << std::setprecision(17)
            << "lb-context-v1|" << robot_id_ << '|'
            << cycle_revision_ << '|' << cycle_cost_revision_ << '|'
            << rx_ << '|' << ry_ << '|' << yaw_ << '|'
            << goal_tolerance_m_ << '|'
            << cost_only_reference_linear_speed_mps_ << '|';
    for (const auto & diagnostic : region_diagnostics_) {
      const auto & region = diagnostic.region;
      payload << diagnostic.id << '|' << diagnostic.status << '|'
              << region.size << '|'
              << region.centroid.first << '|' << region.centroid.second << '|';
      if (region.goal_point) {
        payload << region.goal_point->first << '|' << region.goal_point->second;
      } else {
        payload << "no-goal";
      }
      payload << '|' << diagnostic.approach_x << '|' << diagnostic.approach_y
              << '|' << diagnostic.visible_reveal_gain << '|'
              << diagnostic.optimistic_cost_lower_bound_s << '|';
      append_region_cells_fingerprint(payload, region, 0);
      payload << '|';
    }
    const auto serialized = payload.str();
    const auto hash = fnv1a64(serialized.data(), serialized.size());
    std::ostringstream output;
    output << std::hex << hash;
    return output.str();
  }

  void publish_batch(bool geometry_only = false)
  {
    TimingScope timing(this, TimingSection::PUBLISH_BATCH, now().seconds());
    // Planner callbacks complete asynchronously. Recompute the aggregate
    // evidence from the final per-region states so a queried region cannot
    // remain counted as DETECTED_NOT_QUERIED after it became reachable.
    recount_region_statuses();
    apply_upstream_route_ordering();
    previous_identity_references_.clear();
    if (cycle_map_) {
      frontier_exploration_ros2::OccupancyGrid2d grid(cycle_map_);
      for (const auto & diagnostic : region_diagnostics_) {
        previous_identity_references_.push_back({
          diagnostic.id, diagnostic.region.centroid.first, diagnostic.region.centroid.second,
          frontier_world_bounds(diagnostic.region, grid)});
      }
    }
    my_epuck_interfaces::msg::FrontierCandidateArray message;
    message.header.frame_id = global_frame_;
    message.header.stamp = now();
    message.source_robot_id = robot_id_;
    // Cost-only minimal coordination needs a shared map-content identity;
    // retain the ordered counter for the legacy route-aware mode.
    message.map_revision = selection_policy_ == "frontier_cost_only" ?
      map_checksum(*cycle_map_) : cycle_revision_;
    message.costmap_revision = cycle_cost_revision_;
    message.map_stamp = cycle_map_->header.stamp;
    message.lower_bound_context_fingerprint = lower_bound_context_fingerprint();
    message.planner_id = planner_id_;
    message.detected_frontier_count = detected_frontier_count_;
    message.small_frontier_count = small_frontier_count_;
    message.out_of_range_frontier_count = out_of_range_frontier_count_;
    message.unreachable_frontier_count = unreachable_frontier_count_;
    message.planner_failure_count = planner_failure_count_;
    message.detected_not_queried_count = detected_not_queried_count_;
    message.unclassified_frontier_count = detected_not_queried_count_;
    message.candidate_generation_id = ++candidate_generation_id_;
    message.diagnostic_regions_json = diagnostic_regions_json();
    message.terminal_frontier_regions_json = terminal_frontier_regions_json();
    visualization_msgs::msg::MarkerArray markers;
    int marker_id = 0;
    if (!geometry_only) {
    for (const auto & work : reachable_) {
      my_epuck_interfaces::msg::FrontierCandidate candidate;
      candidate.frontier_id = work.id;
      candidate.physical_frontier_id = work.id;
      candidate.centroid.x = work.region.centroid.first;
      candidate.centroid.y = work.region.centroid.second;
      candidate.bounding_box_min.x = work.bounds[0];
      candidate.bounding_box_min.y = work.bounds[1];
      candidate.bounding_box_max.x = work.bounds[2];
      candidate.bounding_box_max.y = work.bounds[3];
      candidate.approach_pose = work.pose;
      candidate.cell_count = work.region.size;
      candidate.frontier_length_m = work.frontier_length;
      candidate.information_gain = work.gain;
      candidate.euclidean_distance_m = work.euclid;
      candidate.path_length_m = work.path;
      candidate.heading_change_rad = work.heading;
      candidate.score = work.score;
      candidate.mrtsp_route_rank = work.mrtsp_route_rank;
      candidate.mrtsp_route_generation = work.mrtsp_route_generation;
      candidate.mrtsp_solver = upstream_route_ordering_enabled_ ?
        upstream_mrtsp_solver_ : "";
      candidate.reachability_state = candidate.REACHABLE;
      candidate.local_path_length_m = work.path;
      candidate.local_path_samples = work.path_samples;
      markers.markers.emplace_back();
      auto & marker = markers.markers.back();
      marker.header = message.header;
      marker.ns = "reachable_frontiers";
      marker.id = marker_id++;
      marker.type = marker.SPHERE;
      marker.action = marker.ADD;
      marker.pose = work.pose.pose;
      marker.scale.x = marker.scale.y = .04;
      marker.scale.z = .02;
      marker.color.g = 1.0;
      marker.color.a = .9;
      message.candidates.push_back(std::move(candidate));
    }
    } else if (cycle_map_) {
      frontier_exploration_ros2::OccupancyGrid2d grid(cycle_map_);
      for (const auto & diagnostic : region_diagnostics_) {
        const auto bounds = frontier_world_bounds(diagnostic.region, grid);
        my_epuck_interfaces::msg::FrontierCandidate candidate;
        candidate.frontier_id = diagnostic.id;
        candidate.physical_frontier_id = diagnostic.id;
        candidate.centroid.x = diagnostic.region.centroid.first;
        candidate.centroid.y = diagnostic.region.centroid.second;
        candidate.bounding_box_min.x = bounds[0];
        candidate.bounding_box_min.y = bounds[1];
        candidate.bounding_box_max.x = bounds[2];
        candidate.bounding_box_max.y = bounds[3];
        if (diagnostic.has_approach) {
          candidate.approach_pose.header.frame_id = global_frame_;
          candidate.approach_pose.header.stamp = message.header.stamp;
          candidate.approach_pose.pose.position.x = diagnostic.approach_x;
          candidate.approach_pose.pose.position.y = diagnostic.approach_y;
          const double approach_yaw = std::atan2(
            diagnostic.region.centroid.second - diagnostic.approach_y,
            diagnostic.region.centroid.first - diagnostic.approach_x);
          candidate.approach_pose.pose.orientation.z = std::sin(approach_yaw / 2.0);
          candidate.approach_pose.pose.orientation.w = std::cos(approach_yaw / 2.0);
          candidate.euclidean_distance_m = std::hypot(
            diagnostic.approach_x - rx_, diagnostic.approach_y - ry_);
        }
        candidate.cell_count = diagnostic.region.size;
        candidate.frontier_length_m =
          static_cast<float>(diagnostic.region.size * grid.map().info.resolution);
        candidate.information_gain = diagnostic.visible_reveal_gain;
        candidate.reachability_state = candidate.UNKNOWN;
        candidate.path_length_m = 0.0F;
        candidate.heading_change_rad = 0.0F;
        candidate.score = 0.0F;
        candidate.local_path_length_m = 0.0F;
        message.candidates.push_back(std::move(candidate));
      }
    }
    pub_->publish(message);
    marker_pub_->publish(markers);
    record_cycle(
      cycle_map_ ? cycle_map_->data.size() : 0, detected_frontier_count_,
      message.candidates.size());
    RCLCPP_INFO(
      get_logger(),
      "CANDIDATE_METRICS source=FRONTIER_REACHABILITY revision=%lu costmap_revision=%lu detected=%u reachable=%zu queries=%zu cache_hits=%lu cache_misses=%lu detected_not_queried=%u small=%u out_of_range=%u unreachable=%u planner_failures=%u tier1_pending=%zu tier2_pending=%zu tier1_queries=%zu tier2_queries=%zu",
      cycle_revision_, cycle_cost_revision_, detected_frontier_count_, reachable_.size(), queries_,
      classification_cache_hits_, classification_cache_misses_, detected_not_queried_count_,
      small_frontier_count_, out_of_range_frontier_count_, unreachable_frontier_count_,
      planner_failure_count_, tier1_pending_count_, tier2_pending_count_,
      tier1_queries_issued_, tier2_queries_issued_);
    RCLCPP_INFO(
      get_logger(), "FRONTIER_CACHE_SUMMARY classification_hits=%lu classification_misses=%lu path_invalidations=%lu",
      classification_cache_hits_, classification_cache_misses_, path_cache_invalidations_);
  }

  bool suppressed(uint64_t id, uint64_t revision)
  {
    const auto it = suppression_.find(id);
    if (it == suppression_.end()) {return false;}
    if (it->second.revision != revision && it->second.expires_ns == 0) {
      suppression_.erase(it);
      return false;
    }
    if (it->second.expires_ns && now().nanoseconds() > it->second.expires_ns) {
      suppression_.erase(it);
      return false;
    }
    return true;
  }

  void suppress(uint64_t id, uint64_t revision, bool until_revision)
  {
    if (suppression_.size() >= static_cast<std::size_t>(maximum_suppression_records_)) {
      suppression_.erase(suppression_.begin());
    }
    suppression_[id] = {revision, until_revision ? 0 : now().nanoseconds() +
      static_cast<int64_t>(unreachable_suppression_s_ * 1e9)};
  }

  std::mutex mu_;
  std::atomic<State> state_{State::WAITING_FOR_INPUTS};
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr latest_map_, latest_cost_, cycle_map_, cycle_cost_;
  uint64_t map_sum_{0}, cost_sum_{0}, revision_{0}, cost_revision_{0};
  uint64_t core_map_revision_{0}, core_costmap_revision_{0};
  uint64_t cycle_revision_{0}, cycle_cost_revision_{0}, stale_results_{0};
  std::atomic<uint64_t> request_generation_{0}, active_request_{0};
  uint64_t query_event_sequence_{0};
  uint64_t candidate_generation_id_{0};
  uint64_t map_receipts_{0}, map_changed_{0};
  uint64_t cost_receipts_{0}, cost_changed_{0}, classification_cache_hits_{0};
  uint64_t classification_cache_misses_{0}, path_cache_invalidations_{0};
  int64_t last_map_receipt_ns_{0}, last_cost_receipt_ns_{0};
  bool pending_{false};
  double rx_{0.0}, ry_{0.0}, yaw_{0.0}, extract_ms_{0.0};
  std::vector<Work> works_, reachable_;
  std::vector<RegionDiagnostic> region_diagnostics_;
  std::vector<IdentityReference> previous_identity_references_;
  std::unordered_map<uint64_t, EvaluationCache> evaluation_cache_;
  std::unique_ptr<frontier_exploration_ros2::FrontierExplorerCore> core_;
  std::size_t query_index_{0}, queries_{0}, safe_approach_rejections_{0};
  std::size_t tier1_pending_count_{0}, tier2_pending_count_{0};
  std::size_t tier1_queries_issued_{0}, tier2_queries_issued_{0};
  uint32_t detected_frontier_count_{0}, small_frontier_count_{0};
  uint32_t out_of_range_frontier_count_{0}, unreachable_frontier_count_{0};
  uint32_t planner_failure_count_{0}, detected_not_queried_count_{0};
  uint64_t autonomous_dispatch_attempts_{0};
  std::unordered_map<uint64_t, Suppression> suppression_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp_action::Client<Action>::SharedPtr planner_;
  GoalHandle::SharedPtr active_;
  rclcpp::TimerBase::SharedPtr timer_, timeout_timer_, retry_timer_, receipt_summary_timer_;
  std::mutex timeout_timer_mu_;
  std::atomic<uint64_t> timeout_timer_request_{0};
  rclcpp::Subscription<my_epuck_interfaces::msg::RelativePoseHypothesis>::SharedPtr handoff_subscription_;
  rclcpp::Subscription<my_epuck_interfaces::msg::DistributedExplorationStatus>::SharedPtr
    coordinator_status_subscription_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_, cost_sub_;
  rclcpp::Publisher<my_epuck_interfaces::msg::FrontierCandidateArray>::SharedPtr pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  int path_lock_fd_{-1};
  std::string robot_id_, map_topic_, global_costmap_topic_, global_frame_, robot_base_frame_;
  std::string compute_path_action_, candidate_topic_, marker_topic_, path_query_lock_path_,
    path_priority_path_;
  std::string grid_subscription_reliability_, grid_subscription_durability_;
  std::string planner_id_;
  std::string selection_policy_;
  double processing_rate_hz_, minimum_frontier_length_m_, stable_id_quantization_m_;
  bool handoff_gated_{false};
  bool stop_after_handoff_{false};
  bool event_driven_costing_{false};
  bool require_known_approach_{false};
  bool processing_active_{false};
  bool status_gate_seen_{false};
  bool last_costing_request_{false};
  std::string last_costing_union_hash_;
  std::atomic_bool costing_epoch_open_{false};
  uint64_t costing_epoch_id_{0};
  double last_alternative_request_time_s_{-1.0};
  uint8_t last_coordinator_state_{0};
  double approach_clearance_m_, frontier_goal_stepback_m_, planner_tolerance_m_, minimum_robot_distance_m_;
  double path_query_timeout_s_, gain_weight_, distance_weight_;
  double path_weight_, heading_weight_, unreachable_suppression_s_, goal_tolerance_m_;
  double cost_only_reference_linear_speed_mps_, cost_only_reference_angular_speed_radps_;
  bool upstream_route_ordering_enabled_{false};
  std::string upstream_mrtsp_solver_;
  int upstream_mrtsp_candidate_limit_{8};
  int upstream_mrtsp_planning_horizon_{5};
  uint64_t mrtsp_route_generation_{0};
  double visible_gain_range_m_, visible_gain_fov_deg_, visible_gain_ray_step_deg_;
  double classification_context_radius_m_, path_context_radius_m_;
  int occupied_threshold_, costmap_blocked_threshold_, minimum_frontier_cells_;
  bool frontier_map_optimization_enabled_{true};
  double sigma_s_{2.0}, sigma_r_{30.0};
  int dilation_kernel_radius_cells_{1};
  int maximum_candidates_before_path_check_, maximum_path_queries_per_cycle_;
  int maximum_suppression_records_;
  int maximum_evaluation_records_;
  bool forensic_clearance_cells_{false}, diagnostic_frontier_capture_{false};
  bool diagnostic_timing_{false}, timing_summary_emitted_{false};
  std::array<std::array<TimingStats, 2>, static_cast<std::size_t>(TimingSection::COUNT)>
    timing_stats_{};
  std::array<CycleStats, 2> cycle_stats_{};
  uint64_t cycle_sequence_{0};
  double cycle_start_sim_time_{0.0};
  std::size_t cycle_cache_before_{0}, cycle_new_ids_{0}, cycle_reused_ids_{0};
  std::size_t cycle_pruned_absent_{0}, cycle_evicted_capacity_{0};
  std::size_t cycle_identity_associations_{0};
  std::size_t cycle_unique_frontier_records_{0}, cycle_duplicate_id_records_{0};
  std::size_t cycle_tier1_pending_start_{0}, cycle_tier1_selected_{0};
  std::size_t cycle_tier1_sent_{0}, cycle_tier1_reachable_{0};
  std::size_t cycle_tier1_unreachable_{0}, cycle_tier1_aborted_{0};
  std::size_t cycle_tier1_timeout_{0}, cycle_tier1_remaining_dnu_{0};
  std::size_t cycle_map_cancelled_tier1_{0};
  std::size_t cycle_stale_query_rejections_{0};
  uint64_t cycle_old_revision_{0}, cycle_new_revision_{0};
  uint64_t cycle_old_cost_revision_{0}, cycle_new_cost_revision_{0};
  std::string cycle_termination_reason_{"OTHER"};
  uint64_t path_lock_acquire_attempts_{0}, path_lock_retry_count_{0};
  uint64_t path_lock_hold_count_{0}, path_priority_yields_{0};
  double path_lock_wait_total_s_{0.0}, path_lock_wait_max_s_{0.0};
  double path_lock_hold_total_s_{0.0}, path_lock_hold_max_s_{0.0};
  bool path_lock_waiting_{false};
  std::chrono::steady_clock::time_point path_lock_wait_started_{};
  std::chrono::steady_clock::time_point path_lock_acquired_at_{};
  uint64_t path_lock_request_{0};
  uint64_t retry_generation_{0};
};

}  // namespace my_epuck_frontier_candidates

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<my_epuck_frontier_candidates::Generator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
}
