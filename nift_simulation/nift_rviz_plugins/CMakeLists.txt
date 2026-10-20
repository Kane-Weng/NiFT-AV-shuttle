cmake_minimum_required(VERSION 3.10)
project(nift_rviz_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Qt5 setup for the UI
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)
set(CMAKE_AUTOMOC ON) # Crucial for Qt's Q_OBJECT macros
set(CMAKE_AUTOUIC ON)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# RViz panels used with any simulation backend and on the shuttle
add_library(nift_rviz_plugins SHARED
  src/nav_panel.cpp
  include/nift_rviz_plugins/nav_panel.hpp
  src/tf_monitor_panel.cpp
  include/nift_rviz_plugins/tf_monitor_panel.hpp
  src/node_checker_panel.cpp
  include/nift_rviz_plugins/node_checker_panel.hpp
)

target_include_directories(nift_rviz_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(nift_rviz_plugins
  rclcpp
  rviz_common
  std_srvs
  std_msgs
  pluginlib
  diagnostic_msgs
  geometry_msgs
  tf2
  tf2_ros
)
target_link_libraries(nift_rviz_plugins Qt5::Widgets)

# Export the plugin XML so RViz can find it
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

install(TARGETS nift_rviz_plugins
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
