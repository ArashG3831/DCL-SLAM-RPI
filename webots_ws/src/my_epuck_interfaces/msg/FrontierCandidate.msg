uint8 UNKNOWN=0
uint8 REACHABLE=1
uint64 frontier_id
geometry_msgs/Point centroid
geometry_msgs/Point bounding_box_min
geometry_msgs/Point bounding_box_max
geometry_msgs/PoseStamped approach_pose
uint32 cell_count
float32 frontier_length_m
float32 information_gain
float32 euclidean_distance_m
float32 path_length_m
float32 heading_change_rad
float32 score
# Route context from the upstream MRTSP solver.  ``score`` remains the
# legacy per-batch diagnostic only; it must not be used as a cross-robot
# utility by the route-aware distributed mode.
uint32 mrtsp_route_rank
uint64 mrtsp_route_generation
string mrtsp_solver
uint8 reachability_state
float32 local_path_length_m
geometry_msgs/Point[] local_path_samples
# Complete planner result used by the physical solo FollowPath path.
nav_msgs/Path planned_path
# Stable physical identity computed from frontier geometry in the local map.
# It is not a transient array index and is valid for diagnostic/task matching.
uint64 physical_frontier_id
