cmake_minimum_required(VERSION 3.8)
project(cybair_6dof_gripper_control_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(std_msgs REQUIRED)

# teleop_gripper_key
add_executable(teleop_gripper_key src/teleop_gripper_key.cpp)

target_include_directories(
  teleop_gripper_key PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(teleop_gripper_key PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  teleop_gripper_key
  "rclcpp" 
  "std_msgs"
  "moveit_ros_planning_interface"
)

# teleop_gripper_publisher
add_executable(teleop_gripper_publisher src/teleop_gripper_publisher.cpp)
ament_target_dependencies(teleop_gripper_publisher rclcpp std_msgs)

target_include_directories(
  teleop_gripper_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  )
target_compile_features(teleop_gripper_publisher PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  teleop_gripper_publisher
  "rclcpp" 
  "std_msgs"
  "moveit_ros_planning_interface"
)
install(TARGETS 
  teleop_gripper_key
  teleop_gripper_publisher
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
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
