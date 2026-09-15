std_msgs/Header header
string source_robot_id
unique_identifier_msgs/UUID source_session_id
uint64 source_snapshot_epoch
uint64 source_map_revision
uint64 source_costmap_revision
string source_map_fingerprint
builtin_interfaces/Time task_generation_stamp
builtin_interfaces/Duration validity
PhysicalTask[] tasks
# Exact candidate-generation context used for compact cost lower bounds.
# Empty is permitted when there are no unqueried candidates.
string lower_bound_context_fingerprint
# Copied unchanged from the corresponding FrontierCandidateArray.
uint64 candidate_generation_id
