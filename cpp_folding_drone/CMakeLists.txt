cmake_minimum_required(VERSION 3.8)
project(cpp_folding_drone)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Dependencies ──────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(folding_drone_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# pigpiod_if2: daemon client library — connects to running pigpiod over socket,
# no root required. Part of the same libpigpio-dev package.
find_library(PIGPIOD_IF2_LIBRARY pigpiod_if2 REQUIRED)

# ── attitude_control ──────────────────────────────────────────────────────────
add_executable(attitude_control
  src/attitude_control.cpp
)

target_include_directories(attitude_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(attitude_control
  rclcpp
  px4_msgs
  folding_drone_msgs
  std_msgs
  geometry_msgs
)

# ── offboard_publisher ────────────────────────────────────────────────────────
add_executable(offboard_publisher
  src/offboard_publisher.cpp
)

target_include_directories(offboard_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(offboard_publisher
  rclcpp
  px4_msgs
  folding_drone_msgs
  std_msgs
)

# ── joint_angle_publisher ─────────────────────────────────────────────────────
add_executable(joint_angle_publisher
  src/joint_angle_publisher.cpp
)

target_include_directories(joint_angle_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(joint_angle_publisher
  rclcpp
  folding_drone_msgs
  std_msgs
)

# Link against pigpiod daemon client library (NOT the raw -lpigpio)
target_link_libraries(joint_angle_publisher
  ${PIGPIOD_IF2_LIBRARY}
)

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS
  attitude_control
  offboard_publisher
  joint_angle_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
