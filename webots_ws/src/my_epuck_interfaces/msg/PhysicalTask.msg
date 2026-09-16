string source_robot_id
unique_identifier_msgs/UUID source_session_id
uint64 source_snapshot_epoch
uint64 source_map_revision
string physical_signature
uint64 local_frontier_id
geometry_msgs/Point centroid
geometry_msgs/Point bounding_box_min
geometry_msgs/Point bounding_box_max
geometry_msgs/PoseStamped approach_pose
geometry_msgs/Point[] frontier_geometry
geometry_msgs/Point[] visible_cells
geometry_msgs/Point visible_bounds_min
geometry_msgs/Point visible_bounds_max
bool has_visible_bounds
float32 visible_reveal_gain
float32 local_ordering_score
# Route context copied verbatim from the source candidate batch.  Rank zero is
# the upstream solver's next preferred target; UINT32_MAX means not included
# in its bounded route horizon.
uint32 mrtsp_route_rank
uint64 mrtsp_route_generation
string mrtsp_solver
bool local_path_valid
float32 local_path_length_m
geometry_msgs/Point[] local_path_samples
# Complete planner result copied from the source FrontierCandidate.
nav_msgs/Path planned_path
# Initial direction mismatch of the actual valid Nav2 path, when available.
# This is a diagnostic/cost-only primitive; MRTSP route context remains
# represented by the fields above and is not replaced by this value.
float32 path_heading_cost_rad
builtin_interfaces/Time generation_stamp
