cmake_minimum_required(VERSION 2.8.3)
project(invisible_humans_detection)

add_compile_options(-std=c++17)

set(CMAKE_BUILD_TYPE Release)

# # Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  angles
  costmap_2d
  geometry_msgs
  roscpp
  rospy
  tf2_ros
  tf
  tf2_eigen
  nav_msgs
  cohan_msgs
  costmap_converter
)

# Eigen3 FindScript Backward compatibility (ubuntu saucy)
# Since FindEigen.cmake is deprecated starting from jade.
if(EXISTS "FindEigen3.cmake")
  find_package(Eigen3 REQUIRED)
  set(Eigen_INCLUDE_DIRS ${Eigen3_INCLUDE_DIRS})
elseif(EXISTS "FindEigen.cmake")
  find_package(Eigen REQUIRED)
elseif(EXISTS "FindEigen.cmake")
  message(WARNING "No findEigen cmake script found. You must provde one of them,
  e.g. by adding it to ${PROJECT_SOURCE_DIR}/cmake_modules.")
endif(EXISTS "FindEigen3.cmake")

catkin_package(
  INCLUDE_DIRS include ${EXTERNAL_INCLUDE_DIRS}
  LIBRARIES invisible_humans_detection ${EXTERNAL_LIBS}
  CATKIN_DEPENDS angles costmap_2d geometry_msgs roscpp rospy nav_msgs
)

# # Specify additional locations of header files
include_directories(
  include
  ${EXTERNAL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# # Declare a C++ library
add_library(${PROJECT_NAME}
  src/invisible_humans_detection.cpp
)

add_executable(${PROJECT_NAME}_node src/invisible_humans_detection.cpp)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${EXTERNAL_LIBS}
  ${catkin_LIBRARIES}
)
