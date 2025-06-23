cmake_minimum_required(VERSION 3.1.8)
cmake_policy(SET CMP0057 NEW)

project(cohan_sim)
option(BUILD_LIBSIM "Build libsim shared library" ON)

include(FindPkgConfig)
if(DEFINED ENV{ROS_VERSION} AND "$ENV{ROS_VERSION}" STREQUAL "2")
  # ROS 2 setup
  find_package(ament_cmake REQUIRED)
  find_package(rclpy REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(tf2_ros REQUIRED)
  find_package(ament_cmake_python REQUIRED)


  pkg_check_modules(SDL2 REQUIRED sdl2)

  include_directories(
    ${SDL2_INCLUDE_DIRS}
  )

  # Create lib/ directory
  file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/lib)

  if(BUILD_LIBSIM)
    add_custom_command(
      OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/lib/libsim.so
      COMMAND gcc -shared -fPIC -o ${CMAKE_CURRENT_SOURCE_DIR}/lib/libsim.so ${CMAKE_CURRENT_SOURCE_DIR}/src/sim.c -lSDL2 -lm -lyaml
      DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/sim.c
      COMMENT "Building libsim.so"
    )

    add_custom_target(build_sim_lib ALL
      DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/lib/libsim.so
    )

    install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/lib/libsim.so
      DESTINATION lib
    )
  endif()

  # Install the already existing libsim in ros2
  install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/lib/libsim.so
    DESTINATION share/${PROJECT_NAME}/lib
  )

  install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME}/
  )

  install(DIRECTORY maps
    DESTINATION share/${PROJECT_NAME}/
  )

  ament_python_install_package(${PROJECT_NAME})

  install(PROGRAMS
    src/run_sim.py
    DESTINATION lib/${PROJECT_NAME}
  )

  add_executable(run_sim src/run_sim.cpp)
  target_link_libraries(run_sim ${catkin_LIBRARIES} ${SDL2_LIBRARIES})
  install(TARGETS run_sim DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

  ament_package()

else()
  # ROS 1 setup
  find_package(catkin REQUIRED COMPONENTS
    rospy
    nav_msgs
    geometry_msgs
    sensor_msgs
    tf2_ros
  )

  pkg_check_modules(SDL2 REQUIRED sdl2)

  catkin_package()

  include_directories(
    ${catkin_INCLUDE_DIRS}
    ${SDL2_INCLUDE_DIRS}
  )

  file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/lib)

  if(BUILD_LIBSIM)
    add_custom_command(
      OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/lib/libsim.so
      COMMAND gcc -shared -fPIC -o ${CMAKE_CURRENT_SOURCE_DIR}/lib/libsim.so ${CMAKE_CURRENT_SOURCE_DIR}/src/sim.c -lSDL2 -lm -lyaml
      DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/sim.c
      COMMENT "Building libsim.so"
    )

    add_custom_target(build_sim_lib ALL
      DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/lib/libsim.so
    )

    install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/lib/libsim.so
      DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    )
  endif()

  catkin_install_python(PROGRAMS
    src/run_sim.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

  add_executable(run_sim src/run_sim.cpp)
  target_link_libraries(run_sim ${catkin_LIBRARIES} ${SDL2_LIBRARIES})
  install(TARGETS run_sim DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  install(TARGETS run_sim RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

endif()
