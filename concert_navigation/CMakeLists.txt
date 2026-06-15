cmake_minimum_required(VERSION 3.8)
project(concert_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav2_mppi_controller REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

add_library(concert_mppi_critics SHARED
  src/lateral_motion_critic.cpp
)
target_include_directories(concert_mppi_critics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(concert_mppi_critics PUBLIC cxx_std_17)
ament_target_dependencies(concert_mppi_critics
  nav2_mppi_controller
  pluginlib
  rclcpp
)
pluginlib_export_plugin_description_file(nav2_mppi_controller concert_mppi_critics.xml)

install(TARGETS concert_mppi_critics
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config behavior_tree
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY real_sim/
  DESTINATION share/${PROJECT_NAME}/real_sim
  PATTERN "launch" EXCLUDE
)

install(PROGRAMS
  scripts/nav2_debug_capture.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(concert_mppi_critics)
ament_export_dependencies(nav2_mppi_controller pluginlib rclcpp)

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

ament_package()
