cmake_minimum_required(VERSION 2.8.3)
project(cohan_msgs)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_COMPILER_ARG1 -std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  geometry_msgs
  nav_msgs
  tf
  visualization_msgs
)

# # add message, service and action files
add_message_files(DIRECTORY msg
  FILES
  AgentPath.msg
  AgentPathArray.msg
  AgentTimeToGoal.msg
  AgentTimeToGoalArray.msg
  AgentTrajectory.msg
  AgentTrajectoryArray.msg
  AgentType.msg
  TrackedSegment.msg
  TrackedAgent.msg
  TrackedAgents.msg
  TrackedSegmentType.msg
  Trajectory.msg
  TrajectoryStamped.msg;
  TrajectoryArray.msg
  TrajectoryPoint.msg
  StateArray.msg
  AgentMarker.msg
  AgentMarkerStamped.msg
  PointArray.msg
  PassageType.msg
  Predictions.msg
)

add_service_files(
  FILES
  Optimize.srv
  GetTrajectory.srv
)

# # generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  nav_msgs
  std_msgs
)


# # declare catkin package
catkin_package(
  CATKIN_DEPENDS
  roscpp
  message_runtime
  geometry_msgs
  nav_msgs
  tf
  visualization_msgs
)
