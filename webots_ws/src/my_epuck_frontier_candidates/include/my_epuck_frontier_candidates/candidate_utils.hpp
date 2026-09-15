#pragma once
#include <cstdint>
#include <array>
#include <optional>
#include <string>
#include <vector>
#include <nav_msgs/msg/path.hpp>
#include "frontier_exploration_ros2/frontier_search.hpp"
namespace my_epuck_frontier_candidates {
struct ClearanceEvidence {
  bool in_bounds{false}; bool accepted{false}; int column{-1},row{-1};
  int nearest_column{-1},nearest_row{-1},nearest_cost{-1};
  double nearest_distance_m{0.0}; size_t examined_count{0};
  std::vector<std::string> examined_cells;
};
struct FrontierEvaluationRecord {
  uint64_t id{0};
  bool never_queried{true};
  bool invalidated{false};
  bool transient_failure{false};
  uint64_t cycles_not_queried{0};
  int64_t last_query_ns{0};
  // Optimistic frontier_cost_only motion-cost lower bound.  The default keeps
  // source compatibility for callers that do not provide pre-query geometry.
  double optimistic_cost_lower_bound_s{0.0};
  // Tier 1 means the frontier has no current valid path evaluation and must
  // receive first-evaluation service before refresh work can consume slots.
  bool tier1_unqueried{false};
};

// Deterministic bounded two-tier scheduling: first-evaluation work is always
// preferred to path/context refresh work; bounds and starvation age order each
// tier. The limits remain the existing candidate/query budgets.
std::vector<std::size_t> fair_frontier_query_order(
  const std::vector<FrontierEvaluationRecord> & records,
  std::size_t candidate_limit,
  std::size_t query_limit);
uint64_t fnv1a64(const void *data,size_t size,uint64_t seed=14695981039346656037ULL);
uint64_t map_checksum(const nav_msgs::msg::OccupancyGrid &map);
uint64_t local_context_checksum(
  const frontier_exploration_ros2::OccupancyGrid2d & map,
  double world_x, double world_y, double radius_m);
uint64_t stable_frontier_id(const frontier_exploration_ros2::FrontierCandidate &f,const frontier_exploration_ros2::OccupancyGrid2d &map,double quantum);
// Defensive extraction of a metric from a successful Nav2 path. This helper
// checks that the path is non-empty and finite, but does not impose an
// additional endpoint-distance reachability rule.
std::optional<double> extract_nav2_path_cost(const nav_msgs::msg::Path &path);
std::optional<double> path_length(const nav_msgs::msg::Path &path,double robot_x,double robot_y,double goal_x,double goal_y,double tolerance);
std::optional<double> path_initial_heading_cost(
  const nav_msgs::msg::Path &path, double robot_yaw, double minimum_segment_m = 0.05);
// Nominal, physically scaled frontier travel cost.  This is not a Nav2/RPP
// ETA: it combines translation time and the initial reorientation time using
// explicit reference motion limits.
std::optional<double> nominal_motion_cost_s(
  double path_length_m, double heading_cost_rad,
  double reference_linear_speed_mps, double reference_angular_speed_radps);
bool clearance_ok(const frontier_exploration_ros2::OccupancyGrid2d &map,double wx,double wy,double clearance,int blocked_threshold);
ClearanceEvidence clearance_evidence(const frontier_exploration_ros2::OccupancyGrid2d &map,double wx,double wy,double clearance,int blocked_threshold,double trace_radius=0.18,bool capture_cells=false);
bool inside_with_margin(const frontier_exploration_ros2::OccupancyGrid2d &map,double wx,double wy,double margin);
std::optional<std::pair<int, int>> find_safe_approach(
  const frontier_exploration_ros2::OccupancyGrid2d &map,
  double target_x,double target_y,
  double frontier_x,double frontier_y,double search_radius,double inward_margin,
  double clearance,int blocked_threshold);
// Physical solo-fallback variant. It keeps the existing costmap search and
// scoring, but requires the selected point to be known/free in the occupancy
// map before it becomes a navigation approach pose.
std::optional<std::pair<int, int>> find_safe_approach_known_free(
  const frontier_exploration_ros2::OccupancyGrid2d &occupancy_map,
  const frontier_exploration_ros2::OccupancyGrid2d &costmap,
  double target_x,double target_y,
  double frontier_x,double frontier_y,double search_radius,double inward_margin,
  double clearance,int occupancy_threshold,int blocked_threshold);
double normalized_value(double value,double minimum,double maximum);
std::array<double, 4> frontier_world_bounds(
  const frontier_exploration_ros2::FrontierCandidate & frontier,
  const frontier_exploration_ros2::OccupancyGrid2d & map);
bool async_request_is_current(uint64_t request_generation,uint64_t active_request,
  uint64_t request_revision,uint64_t cycle_revision,bool path_checking);
bool candidate_cycle_revisions_match(
  uint64_t request_map_revision, uint64_t current_map_revision,
  uint64_t request_costmap_revision, uint64_t current_costmap_revision);
bool candidate_query_contexts_match(
  uint64_t request_map_context, uint64_t current_map_context,
  uint64_t request_cost_context, uint64_t current_cost_context,
  uint64_t request_path_map_context, uint64_t current_path_map_context,
  uint64_t request_path_cost_context, uint64_t current_path_cost_context);
bool candidate_cycle_retry_ready(
  bool processing_active, bool cycle_idle, bool has_map, bool has_costmap);
bool candidate_retry_callback_is_current(
  uint64_t callback_generation, uint64_t current_generation);
}
