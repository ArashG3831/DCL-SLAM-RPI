std_msgs/Header header
string source_robot_id
uint64 map_revision
uint64 costmap_revision
builtin_interfaces/Time map_stamp
string planner_id
FrontierCandidate[] candidates
uint32 detected_frontier_count
uint32 small_frontier_count
uint32 out_of_range_frontier_count
uint32 unreachable_frontier_count
uint32 planner_failure_count
uint32 unclassified_frontier_count
# Regions emitted by the extractor but not queried in this meaningful state.
uint32 detected_not_queried_count
# Compact diagnostic-only JSON. Empty in ordinary production launches.
string diagnostic_regions_json
# Compact geometry/status summary used by mission termination even when the
# full diagnostic cell capture is disabled.
string terminal_frontier_regions_json
# Compact provenance for the lower-bound summary. This is required allocation
# evidence, not diagnostic-only data.
string lower_bound_context_fingerprint
# Monotonic source-local ID for this exact candidate-generation snapshot.
uint64 candidate_generation_id
