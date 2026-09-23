#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>

#include <chrono>
#include <memory>
#include <thread>
#include <vector>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <cmath>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared(
    "letter_drawer",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  rclcpp::QoS marker_qos(rclcpp::KeepLast(1));
  marker_qos.transient_local();

  auto marker_publisher =
    node->create_publisher<visualization_msgs::msg::Marker>(
      "letter_marker", marker_qos);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::thread spinner([&executor]() {
    executor.spin();
  });

  std::this_thread::sleep_for(std::chrono::seconds(2));

  moveit::planning_interface::MoveGroupInterface move_group(
    node, "ur_manipulator");

  move_group.setPlanningTime(10.0);
  move_group.setNumPlanningAttempts(10);
  move_group.setMaxVelocityScalingFactor(0.15);
  move_group.setMaxAccelerationScalingFactor(0.15);

  RCLCPP_INFO(node->get_logger(), "Da ket noi voi MoveIt");
  move_group.setPoseReferenceFrame("base_link");

  geometry_msgs::msg::Pose start_pose;

  start_pose.orientation.x = 1.0;
  start_pose.orientation.y = 0.0;
  start_pose.orientation.z = 0.0;
  start_pose.orientation.w = 0.0;

  start_pose.position.x = 0.32;
  start_pose.position.y = 0.00;
  start_pose.position.z = 0.30;

  move_group.setPoseTarget(start_pose);

  moveit::planning_interface::MoveGroupInterface::Plan start_plan;

  bool start_success =
    (move_group.plan(start_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (!start_success)
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Khong the lap ke hoach den diem bat dau");

    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(
    node->get_logger(),
    "Da tim thay duong den diem bat dau");

  move_group.execute(start_plan);
  move_group.clearPoseTargets();

  const double center_x = 0.26;
  const double center_y = 0.00;
  const double drawing_z = 0.30;
  const double radius = 0.06;

  const int number_of_points = 72;
  constexpr double pi = 3.14159265358979323846;

  std::vector<geometry_msgs::msg::Pose> waypoints;

  visualization_msgs::msg::Marker marker;

  marker.header.frame_id = "base_link";
  marker.header.stamp = node->now();
  marker.ns = "circle_path";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;

  marker.pose.orientation.w = 1.0;

  marker.scale.x = 0.008;

  marker.color.r = 0.0;
  marker.color.g = 0.2;
  marker.color.b = 1.0;
  marker.color.a = 1.0;

  for (int i = 0; i <= number_of_points; ++i)
  {
    const double angle =
      2.0 * pi * static_cast<double>(i) /
      static_cast<double>(number_of_points);

    const double x = center_x + radius * std::cos(angle);
    const double y = center_y + radius * std::sin(angle);

    geometry_msgs::msg::Point marker_point;
    marker_point.x = x;
    marker_point.y = y;
    marker_point.z = drawing_z;

    marker.points.push_back(marker_point);

    // Điểm i = 0 chính là start_pose nên không cần thêm lần nữa.
    if (i > 0)
    {
      geometry_msgs::msg::Pose circle_pose = start_pose;

      circle_pose.position.x = x;
      circle_pose.position.y = y;
      circle_pose.position.z = drawing_z;

      waypoints.push_back(circle_pose);
    }
  }

  marker_publisher->publish(marker);

  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  
  moveit_msgs::msg::RobotTrajectory trajectory;

  const double eef_step = 0.005;
  const double jump_threshold = 0.0;

  double fraction = move_group.computeCartesianPath(
    waypoints,
    eef_step,
    jump_threshold,
    trajectory,
    true);

  RCLCPP_INFO(
    node->get_logger(),
    "Ti le quy dao tinh duoc: %.1f%%",
    fraction * 100.0);

  int result_code = 0;

  if (fraction < 0.95)
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Quy dao khong du 95 phan tram, huy thuc thi");

    result_code = 1;
  }
  else
  {
    RCLCPP_INFO(
      node->get_logger(),
      "Bat dau ve hinh tron");

    auto execute_result = move_group.execute(trajectory);

    if (execute_result == moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_INFO(
        node->get_logger(),
        "Da ve xong hinh tron");
    }
    else
    {
      RCLCPP_ERROR(
        node->get_logger(),
        "Thuc thi quy dao that bai");

      result_code = 1;
    }
  }

  executor.cancel();
  spinner.join();
  rclcpp::shutdown();

  return result_code;
}
