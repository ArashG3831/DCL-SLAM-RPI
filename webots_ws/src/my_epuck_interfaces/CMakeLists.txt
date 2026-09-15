cmake_minimum_required(VERSION 3.8)
project(my_epuck_interfaces)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(unique_identifier_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FrontierCandidate.msg"
  "msg/FrontierCandidateArray.msg"
  "msg/ExplorationClaim.msg"
  "msg/ExplorationStatus.msg"
  "msg/ExplorationEvent.msg"
  "msg/PeerMap.msg"
  "msg/PhysicalTask.msg"
  "msg/TaskSnapshot.msg"
  "msg/TaskBid.msg"
  "msg/TaskBidArray.msg"
  "msg/PairDecision.msg"
  "msg/DistributedExplorationStatus.msg"
  "msg/ExplorationFailure.msg"
  "msg/DistributedExplorationEvent.msg"
  "msg/LocalMapDescriptor.msg"
  "msg/LocalMapCropRequest.msg"
  "msg/LocalMapCrop.msg"
  "msg/RelativePoseHypothesis.msg"
  "msg/ScanChunk.msg"
  "msg/FullMapSnapshotRequest.msg"
  "msg/FullMapSnapshotResponse.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs nav_msgs std_msgs unique_identifier_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
