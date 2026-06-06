cmake_minimum_required(VERSION 3.8)
project(bot_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(bot_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)


add_executable(vision_frontend
  src/vision_frontend.cpp
  src/camera_context.cpp
  src/correspondence.cpp
  src/distortion_correction.cpp
  src/feature_extraction.cpp
  src/feature_tracks.cpp
  src/multiview_policy.cpp
  src/preprocessing.cpp
  src/triangulation.cpp
  generated/c/casadi/triangulation_sigma_res_staged.c
  src/casADi_helper.cpp
)

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
  set_source_files_properties(
    generated/c/casadi/triangulation_sigma_res_staged.c
    PROPERTIES COMPILE_OPTIONS "-Wno-unused-parameter"
  )
endif()

add_executable(matcher_probe_client
  src/matcher_probe_client.cpp
)

ament_target_dependencies(
  vision_frontend
  rclcpp
  sensor_msgs
  cv_bridge
  bot_interfaces
  geometry_msgs
  tf2_ros
)

ament_target_dependencies(
  matcher_probe_client
  rclcpp
  bot_interfaces
)

target_include_directories(vision_frontend PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/generated/cpp/symforce
  ${CMAKE_CURRENT_SOURCE_DIR}/generated/c/casadi
)

target_link_libraries(vision_frontend
  Eigen3::Eigen
  ${OpenCV_LIBS}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY generated/cpp/symforce
  DESTINATION include
)

install(
  DIRECTORY generated/c/casadi
  DESTINATION include
)

install(TARGETS
  vision_frontend
  matcher_probe_client
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
