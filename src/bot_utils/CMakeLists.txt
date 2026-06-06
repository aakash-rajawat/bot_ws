cmake_minimum_required(VERSION 3.8)
project(bot_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(twist_mux_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(include)

add_executable(bot_trajectory src/bot_trajectory.cpp)
ament_target_dependencies(
  bot_trajectory 
  rclcpp 
  nav_msgs 
  geometry_msgs
)

add_executable(safety_stop src/safety_stop.cpp)
ament_target_dependencies(
  safety_stop 
  rclcpp 
  sensor_msgs 
  std_msgs 
  rclcpp_action 
  twist_mux_msgs 
  visualization_msgs
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  bot_trajectory
  safety_stop
  DESTINATION lib/${PROJECT_NAME}
)


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
