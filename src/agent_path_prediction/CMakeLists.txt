
cmake_minimum_required(VERSION 2.8.3)
project(agent_path_prediction)

set(CMAKE_CXX_COMPILER_ARG1 -std=c++11)
set(CMAKE_BUILD_TYPE Release)

find_package(yaml-cpp REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  geometry_msgs
  cohan_msgs
  message_generation
  nav_msgs
  roscpp
  std_srvs
  tf
  visualization_msgs
  tf2
  tf2_ros
  costmap_2d
)

# # install python modules and global scripts
# catkin_python_setup()

# add message, serviece and action files
add_message_files(DIRECTORY msg
  FILES
  PredictedPoses.msg
  PredictedGoal.msg
  PredictedGoals.msg
  AgentPose.msg
  HumanInfo.msg
  AgentsInfo.msg
)

add_service_files(DIRECTORY srv
  FILES
  AgentPosePredict.srv
  AgentGoal.srv
)

# generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  cohan_msgs
  geometry_msgs
)

# add dynamic reconfigure configs
generate_dynamic_reconfigure_options(
  cfg/AgentPathPrediction.cfg
)

# # declare catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES agent_path_prediction
  CATKIN_DEPENDS
  dynamic_reconfigure
  geometry_msgs
  cohan_msgs
  message_runtime
  nav_msgs
  roscpp
  std_srvs
  tf
  visualization_msgs
  tf2
  tf2_ros
  costmap_2d

  # DEPENDS
  # system_lib
)

# # build ##
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# # Build the library
add_library(agent_path_prediction
  src/agents_class.cpp
  src/predict_goal_ros.cpp
)

# Dynamic reconfigure: make sure configure headers are built before any node using them
add_dependencies(agent_path_prediction ${PROJECT_NAME}_gencfg)

# Generate messages before compiling the lib
add_dependencies(agent_path_prediction ${PROJECT_NAME}_generate_messages_cpp)

add_dependencies(agent_path_prediction ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(agent_path_prediction
  ${EXTERNAL_LIBS}
  ${catkin_LIBRARIES}
  yaml-cpp
)

# declare a c++ executable
add_executable(agent_path_predict
  src/agent_path_prediction.cpp
  src/predict_goal_ros.cpp
)

# cmake target dependencies of the c++ executable
add_dependencies(agent_path_predict ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )

# libraries to link the target c++ executable against
target_link_libraries(agent_path_predict ${EXTERNAL_LIBS} ${catkin_LIBRARIES} yaml-cpp)

install(TARGETS agent_path_prediction
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)