cmake_minimum_required(VERSION 3.5)
project(nav2_carrot_controller)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_2d_msgs REQUIRED)
find_package(dwb_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_2d_utils REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_core REQUIRED)
find_package(dwb_core REQUIRED)

nav2_package()

include_directories(
  include
)

set(dependencies
  rclcpp
  std_msgs
  geometry_msgs
  nav_2d_msgs
  dwb_msgs
  nav2_costmap_2d
  pluginlib
  sensor_msgs
  visualization_msgs
  nav_2d_utils
  nav_msgs
  tf2_ros
  nav2_util
  nav2_core
  dwb_core
)

add_library(nav2_carrot_controller SHARED
  src/carrot_controller.cpp
)

# prevent pluginlib from using boost
target_compile_definitions(nav2_carrot_controller PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

ament_target_dependencies(nav2_carrot_controller
  ${dependencies}
)

install(TARGETS nav2_carrot_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
endif()

ament_export_include_directories(include)
ament_export_libraries(nav2_carrot_controller)
ament_export_dependencies(${dependencies})

pluginlib_export_plugin_description_file(nav2_core local_planner_plugin.xml)

ament_package()
