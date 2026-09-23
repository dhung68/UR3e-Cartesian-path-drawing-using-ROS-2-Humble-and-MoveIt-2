cmake_minimum_required(VERSION 3.8)
project(ur3e_letter_drawer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)

add_executable(letter_drawer_node
  src/letter_drawer.cpp
)

ament_target_dependencies(letter_drawer_node
  rclcpp
  geometry_msgs
  visualization_msgs
  moveit_ros_planning_interface
  moveit_msgs
)

add_executable(circle_drawer_node
  src/circle_drawer.cpp
)

ament_target_dependencies(circle_drawer_node
  rclcpp
  geometry_msgs
  visualization_msgs
  moveit_ros_planning_interface
  moveit_msgs
)

install(TARGETS
  letter_drawer_node
  circle_drawer_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
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
