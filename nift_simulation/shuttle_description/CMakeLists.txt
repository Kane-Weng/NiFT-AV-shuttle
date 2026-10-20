cmake_minimum_required(VERSION 3.10)
project(shuttle_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Nothing is compiled here: the package only installs the URDF (box geometry, no meshes), launch and RViz files.
# The dimensions come from nift_common's vehicle.yaml (loaded by the xacro files).
install(
  DIRECTORY description launch rviz
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
