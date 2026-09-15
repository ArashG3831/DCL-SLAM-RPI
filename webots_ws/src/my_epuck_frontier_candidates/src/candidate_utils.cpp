#include "my_epuck_frontier_candidates/candidate_utils.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <numeric>
#include <tuple>
#include <unordered_set>
#include "frontier_exploration_ros2/frontier_policy.hpp"
namespace my_epuck_frontier_candidates {
std::vector<std::size_t> fair_frontier_query_order(
  const std::vector<FrontierEvaluationRecord> & records,
  std::size_t candidate_limit, std::size_t query_limit)
{
  if (candidate_limit == 0U || query_limit == 0U) {
    return {};
  }
  std::vector<std::size_t> order(records.size());
  std::iota(order.begin(), order.end(), 0U);
  std::stable_sort(order.begin(), order.end(), [&records](std::size_t a, std::size_t b) {
    const auto priority = [&records](std::size_t i) {
      const auto & record = records[i];
      if (record.never_queried) {return 0;}
      if (record.invalidated) {return 1;}
      if (record.transient_failure) {return 2;}
      return 3;
    };
    if (records[a].tier1_unqueried != records[b].tier1_unqueried) {
      return records[a].tier1_unqueried > records[b].tier1_unqueried;
    }
    if (records[a].optimistic_cost_lower_bound_s !=
        records[b].optimistic_cost_lower_bound_s) {
      return records[a].optimistic_cost_lower_bound_s <
             records[b].optimistic_cost_lower_bound_s;
    }
    if (records[a].cycles_not_queried != records[b].cycles_not_queried) {
      return records[a].cycles_not_queried > records[b].cycles_not_queried;
    }
    if (records[a].last_query_ns != records[b].last_query_ns) {
      return records[a].last_query_ns < records[b].last_query_ns;
    }
    if (priority(a) != priority(b)) {return priority(a) < priority(b);}
    return records[a].id < records[b].id;
  });
  // A malformed or transiently colliding candidate batch must not spend more
  // than one bounded query slot on one cache key. Keep the best-ranked record
  // for each ID and apply both existing limits to the deduplicated order.
  std::vector<std::size_t> unique_order;
  unique_order.reserve(std::min(order.size(), query_limit));
  std::unordered_set<uint64_t> selected_ids;
  selected_ids.reserve(order.size());
  for (const auto index : order) {
    if (!selected_ids.insert(records[index].id).second) {
      continue;
    }
    unique_order.push_back(index);
    if (unique_order.size() >= candidate_limit || unique_order.size() >= query_limit) {
      break;
    }
  }
  return unique_order;
}
uint64_t fnv1a64(const void*d,size_t n,uint64_t h){auto*p=static_cast<const uint8_t*>(d);for(size_t i=0;i<n;i++){h^=p[i];h*=1099511628211ULL;}return h;}
template<class T>static void add(uint64_t&h,const T&v){h=fnv1a64(&v,sizeof(v),h);}
uint64_t map_checksum(const nav_msgs::msg::OccupancyGrid&m){uint64_t h=14695981039346656037ULL;add(h,m.info.resolution);add(h,m.info.width);add(h,m.info.height);add(h,m.info.origin.position.x);add(h,m.info.origin.position.y);add(h,m.info.origin.position.z);add(h,m.info.origin.orientation.x);add(h,m.info.origin.orientation.y);add(h,m.info.origin.orientation.z);add(h,m.info.origin.orientation.w);if(!m.data.empty())h=fnv1a64(m.data.data(),m.data.size(),h);return h;}
uint64_t local_context_checksum(const frontier_exploration_ros2::OccupancyGrid2d&m,double wx,double wy,double radius){int cx,cy;uint64_t h=14695981039346656037ULL;if(!m.worldToMapNoThrow(wx,wy,cx,cy)){return fnv1a64(&wx,sizeof(wx),fnv1a64(&wy,sizeof(wy),h));}const int r=std::max(1,int(std::ceil(radius/m.map().info.resolution)));add(h,cx);add(h,cy);for(int y=cy-r;y<=cy+r;y++)for(int x=cx-r;x<=cx+r;x++){if(x<0||y<0||x>=m.getSizeX()||y>=m.getSizeY()){const int value=-2;add(h,value);continue;}const auto value=m.getCost(x,y);add(h,value);}return h;}
uint64_t stable_frontier_id(
  const frontier_exploration_ros2::FrontierCandidate & f,
  const frontier_exploration_ros2::OccupancyGrid2d & map,
  double q)
{
  // The old centroid-only 0.15 m hash merged distinct frontiers that happened
  // to fall in one bin. Keep that coarse centroid for continuity, then add a
  // finer frontier-cell reference and coarse physical bounds. The center point
  // is selected from the frontier cells (not from the robot), while the bounds
  // are intentionally coarser so a small boundary change does not churn the
  // cache key.
  const double centroid_quantum = std::max({3.0 * std::max(q, 1e-6), 0.15});
  const double reference_quantum = std::max({std::max(q, 1e-6), 0.08});
  const double bounds_quantum = std::max({2.0 * std::max(q, 1e-6), 0.12});
  uint64_t h = 14695981039346656037ULL;
  const auto round_quantize = [](double value, double quantum) {
      return static_cast<int64_t>(std::llround(value / quantum));
    };
  const auto floor_quantize = [](double value, double quantum) {
      return static_cast<int64_t>(std::floor(value / quantum));
    };
  add(h, round_quantize(f.centroid.first, centroid_quantum));
  add(h, round_quantize(f.centroid.second, centroid_quantum));
  add(h, floor_quantize(f.center_point.first, reference_quantum));
  add(h, floor_quantize(f.center_point.second, reference_quantum));
  if (f.visible_reveal_bounds) {
    const auto bounds = frontier_world_bounds(f, map);
    add(h, floor_quantize(bounds[0], bounds_quantum));
    add(h, floor_quantize(bounds[1], bounds_quantum));
    add(h, floor_quantize(bounds[2], bounds_quantum));
    add(h, floor_quantize(bounds[3], bounds_quantum));
  } else {
    // Compatibility-constructed candidates do not carry cell bounds. The
    // center-point reference above remains their physical tie-breaker.
    const uint8_t no_bounds = 0U;
    add(h, no_bounds);
  }
  return h;
}
std::optional<double> extract_nav2_path_cost(const nav_msgs::msg::Path &path)
{
  if (path.poses.empty()) {
    return std::nullopt;
  }
  for (const auto & stamped : path.poses) {
    const auto & position = stamped.pose.position;
    const auto & orientation = stamped.pose.orientation;
    if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
        !std::isfinite(position.z) || !std::isfinite(orientation.x) ||
        !std::isfinite(orientation.y) || !std::isfinite(orientation.z) ||
        !std::isfinite(orientation.w)) {
      return std::nullopt;
    }
  }
  double total = 0.0;
  for (std::size_t i = 1; i < path.poses.size(); ++i) {
    total += std::hypot(
      path.poses[i].pose.position.x - path.poses[i - 1].pose.position.x,
      path.poses[i].pose.position.y - path.poses[i - 1].pose.position.y);
  }
  return std::isfinite(total) ? std::optional<double>(total) : std::nullopt;
}

std::optional<double> path_length(
  const nav_msgs::msg::Path & path, double rx, double ry, double gx, double gy, double tol)
{
  const auto total = extract_nav2_path_cost(path);
  if (!total) {
    return std::nullopt;
  }
  const auto & last = path.poses.back().pose.position;
  if (std::hypot(last.x - gx, last.y - gy) > tol) {
    return std::nullopt;
  }
  if (path.poses.size() == 1 && std::hypot(rx - gx, ry - gy) > tol) {
    return std::nullopt;
  }
  return total;
}
std::optional<double> path_initial_heading_cost(
  const nav_msgs::msg::Path & path, double robot_yaw, double minimum_segment_m)
{
  if (path.poses.size() < 2 || !std::isfinite(robot_yaw) ||
      !std::isfinite(minimum_segment_m) || minimum_segment_m <= 0.0) {
    return std::nullopt;
  }
  const auto & first = path.poses.front().pose.position;
  if (!std::isfinite(first.x) || !std::isfinite(first.y)) {
    return std::nullopt;
  }
  for (std::size_t index = 1; index < path.poses.size(); ++index) {
    const auto & second = path.poses[index].pose.position;
    if (!std::isfinite(second.x) || !std::isfinite(second.y)) {
      return std::nullopt;
    }
    const double dx = second.x - first.x;
    const double dy = second.y - first.y;
    if (std::hypot(dx, dy) < minimum_segment_m) {
      continue;
    }
    const double path_yaw = std::atan2(dy, dx);
    const double delta = std::atan2(
      std::sin(path_yaw - robot_yaw), std::cos(path_yaw - robot_yaw));
    return std::abs(delta);
  }
  return std::nullopt;
}
std::optional<double> nominal_motion_cost_s(
  double path_length_m, double heading_cost_rad,
  double reference_linear_speed_mps, double reference_angular_speed_radps)
{
  if (!std::isfinite(path_length_m) || path_length_m < 0.0 ||
      !std::isfinite(heading_cost_rad) || heading_cost_rad < 0.0 ||
      !std::isfinite(reference_linear_speed_mps) || reference_linear_speed_mps <= 0.0 ||
      !std::isfinite(reference_angular_speed_radps) || reference_angular_speed_radps <= 0.0) {
    return std::nullopt;
  }
  const double result = path_length_m / reference_linear_speed_mps +
    heading_cost_rad / reference_angular_speed_radps;
  return std::isfinite(result) ? std::optional<double>(result) : std::nullopt;
}
ClearanceEvidence clearance_evidence(const frontier_exploration_ros2::OccupancyGrid2d&m,double wx,double wy,double clear,int threshold,double trace_radius,bool capture_cells){
  ClearanceEvidence out;int cx,cy;if(!m.worldToMapNoThrow(wx,wy,cx,cy))return out;out.in_bounds=true;out.column=cx;out.row=cy;const double resolution=m.map().info.resolution;const int r=int(std::ceil(std::max(clear,trace_radius)/resolution));out.accepted=true;double best=std::numeric_limits<double>::infinity();
  for(int y=cy-r;y<=cy+r;y++)for(int x=cx-r;x<=cx+r;x++){
    if(x<0||y<0||x>=m.getSizeX()||y>=m.getSizeY()){if(std::hypot((x-cx)*resolution,(y-cy)*resolution)<=clear)out.accepted=false;continue;}
    const auto cost=m.getCost(x,y);const double distance=std::hypot((x-cx)*resolution,(y-cy)*resolution);
    if(distance<=trace_radius){out.examined_count++;if(capture_cells)out.examined_cells.push_back(std::to_string(x)+":"+std::to_string(y)+":"+std::to_string(cost)+":"+std::to_string(distance));}
    if(cost>=threshold&&distance<best){best=distance;out.nearest_column=x;out.nearest_row=y;out.nearest_cost=cost;out.nearest_distance_m=distance;}
    if(distance<=clear&&cost>=threshold)out.accepted=false;
  }
  return out;
}
bool clearance_ok(const frontier_exploration_ros2::OccupancyGrid2d&m,double wx,double wy,double clear,int threshold){return clearance_evidence(m,wx,wy,clear,threshold,clear,false).accepted;}
bool inside_with_margin(const frontier_exploration_ros2::OccupancyGrid2d&m,double wx,double wy,double margin){int x,y;if(!m.worldToMapNoThrow(wx,wy,x,y))return false;const auto&o=m.map().info.origin;const double resolution=m.map().info.resolution;const double a=std::atan2(2*(o.orientation.w*o.orientation.z+o.orientation.x*o.orientation.y),1-2*(o.orientation.y*o.orientation.y+o.orientation.z*o.orientation.z));const double dx=wx-o.position.x,dy=wy-o.position.y;const double gx=std::cos(a)*dx+std::sin(a)*dy,gy=-std::sin(a)*dx+std::cos(a)*dy;return gx>margin&&gy>margin&&gx<m.getSizeX()*resolution-margin&&gy<m.getSizeY()*resolution-margin;}
std::optional<std::pair<int,int>> find_safe_approach(const frontier_exploration_ros2::OccupancyGrid2d&m,double tx,double ty,double fx,double fy,double radius,double margin,double clear,int threshold){int cx,cy;if(!m.worldToMapNoThrow(tx,ty,cx,cy))return{};const double resolution=m.map().info.resolution;const int r=std::max(1,int(std::ceil(radius/resolution)));std::optional<std::pair<int,int>> best;double score=1e99;for(int y=cy-r;y<=cy+r;y++)for(int x=cx-r;x<=cx+r;x++){if(x<0||y<0||x>=m.getSizeX()||y>=m.getSizeY()||m.getCost(x,y)<0||m.getCost(x,y)>=threshold)continue;auto w=m.mapToWorld(x,y);if(std::hypot(w.first-tx,w.second-ty)>radius||!inside_with_margin(m,w.first,w.second,margin)||!clearance_ok(m,w.first,w.second,clear,threshold))continue;double s=std::hypot(w.first-tx,w.second-ty)+.05*std::hypot(w.first-fx,w.second-fy);if(s<score){score=s;best=std::pair<int,int>{x,y};}}return best;}
std::optional<std::pair<int,int>> find_safe_approach_known_free(
  const frontier_exploration_ros2::OccupancyGrid2d &occupancy_map,
  const frontier_exploration_ros2::OccupancyGrid2d &costmap,
  double tx, double ty, double fx, double fy, double radius, double margin,
  double clear, int occupancy_threshold, int blocked_threshold)
{
  int cx, cy;
  if (!costmap.worldToMapNoThrow(tx, ty, cx, cy)) {return {};}
  const double resolution = costmap.map().info.resolution;
  const int r = std::max(1, static_cast<int>(std::ceil(radius / resolution)));
  std::optional<std::pair<int, int>> best;
  double score = std::numeric_limits<double>::infinity();
  for (int y = cy - r; y <= cy + r; ++y) {
    for (int x = cx - r; x <= cx + r; ++x) {
      if (x < 0 || y < 0 || x >= costmap.getSizeX() || y >= costmap.getSizeY()) {
        continue;
      }
      const int costmap_value = costmap.getCost(x, y);
      if (costmap_value < 0 || costmap_value >= blocked_threshold) {continue;}
      const auto world = costmap.mapToWorld(x, y);
      if (std::hypot(world.first - tx, world.second - ty) > radius ||
        !inside_with_margin(costmap, world.first, world.second, margin) ||
        !clearance_ok(costmap, world.first, world.second, clear, blocked_threshold)) {
        continue;
      }
      int map_x, map_y;
      if (!occupancy_map.worldToMapNoThrow(world.first, world.second, map_x, map_y)) {
        continue;
      }
      const int map_value = occupancy_map.getCost(map_x, map_y);
      if (map_value < 0 || map_value >= occupancy_threshold) {continue;}
      const double score_value =
        std::hypot(world.first - tx, world.second - ty) +
        .05 * std::hypot(world.first - fx, world.second - fy);
      if (score_value < score) {
        score = score_value;
        best = std::pair<int, int>{x, y};
      }
    }
  }
  return best;
}
double normalized_value(double v,double lo,double hi){return hi-lo<1e-9?0.:((v-lo)/(hi-lo));}
std::array<double,4> frontier_world_bounds(const frontier_exploration_ros2::FrontierCandidate&f,const frontier_exploration_ros2::OccupancyGrid2d&m){if(!f.visible_reveal_bounds)return {f.centroid.first,f.centroid.second,f.centroid.first,f.centroid.second};const auto&b=*f.visible_reveal_bounds;const std::array<std::pair<int,int>,4> cells={{{b.min_x,b.min_y},{b.min_x,b.max_y},{b.max_x,b.min_y},{b.max_x,b.max_y}}};std::array<double,4> result={std::numeric_limits<double>::infinity(),std::numeric_limits<double>::infinity(),-std::numeric_limits<double>::infinity(),-std::numeric_limits<double>::infinity()};for(const auto&cell:cells){const auto w=m.mapToWorld(cell.first,cell.second);result[0]=std::min(result[0],w.first);result[1]=std::min(result[1],w.second);result[2]=std::max(result[2],w.first);result[3]=std::max(result[3],w.second);}return result;}
bool async_request_is_current(uint64_t request_generation,uint64_t active_request,
  uint64_t request_revision,uint64_t cycle_revision,bool path_checking){
  return request_generation==active_request && request_revision==cycle_revision && path_checking;
}
bool candidate_cycle_revisions_match(
  uint64_t request_map_revision, uint64_t current_map_revision,
  uint64_t request_costmap_revision, uint64_t current_costmap_revision)
{
  return request_map_revision == current_map_revision &&
         request_costmap_revision == current_costmap_revision;
}
bool candidate_query_contexts_match(
  uint64_t request_map_context, uint64_t current_map_context,
  uint64_t request_cost_context, uint64_t current_cost_context,
  uint64_t request_path_map_context, uint64_t current_path_map_context,
  uint64_t request_path_cost_context, uint64_t current_path_cost_context)
{
  return request_map_context == current_map_context &&
         request_cost_context == current_cost_context &&
         request_path_map_context == current_path_map_context &&
         request_path_cost_context == current_path_cost_context;
}
bool candidate_cycle_retry_ready(
  bool processing_active, bool cycle_idle, bool has_map, bool has_costmap)
{
  return processing_active && cycle_idle && has_map && has_costmap;
}
bool candidate_retry_callback_is_current(
  uint64_t callback_generation, uint64_t current_generation)
{
  return callback_generation == current_generation;
}
}
