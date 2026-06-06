cmake_minimum_required(VERSION 3.8)
project(bot_multisensor_odometry)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(bot_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(bot_dugma REQUIRED)

include_directories(include)

add_executable(ua_lidar_point_cloud
  src/ua_lidar_point_cloud.cpp
  src/casadi_helper.cpp
  generated/c/casadi/lidar_point_cloud_staged.c
)

ament_target_dependencies(
  ua_lidar_point_cloud
  rclcpp
  sensor_msgs
  bot_interfaces
  tf2_ros
  geometry_msgs
)

add_executable(relative_pose_odometry
  src/relative_pose_odometry.cpp
)

ament_target_dependencies(
  relative_pose_odometry
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
  bot_interfaces
)

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
  set_source_files_properties(
    generated/c/casadi/lidar_point_cloud_staged.c
    PROPERTIES COMPILE_OPTIONS "-Wno-unused-parameter"
  )
endif()

add_library(mle_relative_pose_client SHARED src/mle_relative_pose_client.cpp)
target_include_directories(mle_relative_pose_client PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_definitions(mle_relative_pose_client
  PRIVATE "MLE_RELATIVE_POSE_CLIENT_CPP_BUILDING_DLL"
)
ament_target_dependencies(mle_relative_pose_client
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "bot_interfaces"
  "message_filters"
  "tf2"
  "tf2_geometry_msgs"
  "geometry_msgs")
rclcpp_components_register_node(mle_relative_pose_client
  PLUGIN "bot_multisensor_odometry::mle_relative_pose_client::MLERelativePoseClient"
  EXECUTABLE mle_relative_pose_client_node
)

add_library(mle_relative_pose_server SHARED src/mle_relative_pose_server.cpp)
target_include_directories(mle_relative_pose_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_definitions(mle_relative_pose_server
  PRIVATE "MLE_RELATIVE_POSE_SERVER_CPP_BUILDING_DLL"
)
ament_target_dependencies(mle_relative_pose_server
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "bot_interfaces"
  "message_filters"
  "bot_dugma")
rclcpp_components_register_node(mle_relative_pose_server
  PLUGIN "bot_multisensor_odometry::mle_relative_pose_server::MLERelativePoseServer"
  EXECUTABLE mle_relative_pose_server_node
)

target_include_directories(ua_lidar_point_cloud PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/generated/c/casadi
)

target_include_directories(mle_relative_pose_client PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(mle_relative_pose_client
  Eigen3::Eigen
)

target_include_directories(mle_relative_pose_server PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(ua_lidar_point_cloud
  Eigen3::Eigen
)

target_link_libraries(mle_relative_pose_server
  Eigen3::Eigen
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY generated/c/casadi
  DESTINATION include
)

install(TARGETS
  ua_lidar_point_cloud
  relative_pose_odometry
  DESTINATION lib/${PROJECT_NAME}
)

INSTALL(TARGETS
  mle_relative_pose_client
  mle_relative_pose_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
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
