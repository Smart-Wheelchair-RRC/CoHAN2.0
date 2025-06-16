include(CMakeFindDependencyMacro)

find_dependency(Eigen3)
find_dependency(OpenGL)

# Find spdlog if g2o was build with support for it
if ()
  find_dependency(spdlog)
endif()

include("${CMAKE_CURRENT_LIST_DIR}/g2oTargets.cmake")
