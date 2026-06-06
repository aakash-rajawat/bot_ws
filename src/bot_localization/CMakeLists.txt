cmake_minimum_required(VERSION 3.8)
project(bot_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(bot_interfaces REQUIRED)

include_directories(include)

add_executable(imu_republisher src/imu_republisher.cpp)
ament_target_dependencies(
  imu_republisher rclcpp 
  sensor_msgs
)

add_executable(odometry_ekf src/odometry_ekf.cpp)
ament_target_dependencies(
  odometry_ekf
  rclcpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
)

add_executable(gtsam_pose_slam src/gtsam_pose_slam.cpp)
ament_target_dependencies(
  gtsam_pose_slam
  rclcpp
  bot_interfaces
  nav_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
)
target_link_libraries(gtsam_pose_slam gtsam Eigen3::Eigen)

target_link_libraries(odometry_ekf Eigen3::Eigen)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  imu_republisher
  odometry_ekf
  gtsam_pose_slam
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
