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

  start_pose.position.x = 0.20;
  start_pose.position.y = -0.06;
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

  const double letter_height = 0.12;
  const double letter_width = 0.12;
  const double lift_height = 0.04;

  visualization_msgs::msg::Marker marker;

  marker.header.frame_id = "base_link";
  marker.header.stamp = node->now();
  marker.ns = "letter_h";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;

  marker.pose.orientation.w = 1.0;

  marker.scale.x = 0.008;

  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;

  auto add_line =
    [&marker](
      double x1, double y1, double z1,
      double x2, double y2, double z2)
    {
      geometry_msgs::msg::Point point_1;
      point_1.x = x1;
      point_1.y = y1;
      point_1.z = z1;

      geometry_msgs::msg::Point point_2;
      point_2.x = x2;
      point_2.y = y2;
      point_2.z = z2;

      marker.points.push_back(point_1);
      marker.points.push_back(point_2);
    };

  // Net doc ben trai
  add_line(
    start_pose.position.x,
    start_pose.position.y,
    start_pose.position.z,
    start_pose.position.x + letter_height,
    start_pose.position.y,
    start_pose.position.z);

  // Net doc ben phai
  add_line(
    start_pose.position.x,
    start_pose.position.y + letter_width,
    start_pose.position.z,
    start_pose.position.x + letter_height,
    start_pose.position.y + letter_width,
    start_pose.position.z);

  // Net ngang o giua
  add_line(
    start_pose.position.x + letter_height / 2.0,
    start_pose.position.y,
    start_pose.position.z,
    start_pose.position.x + letter_height / 2.0,
    start_pose.position.y + letter_width,
    start_pose.position.z);

  marker_publisher->publish(marker);

  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  std::vector<geometry_msgs::msg::Pose> waypoints;
  geometry_msgs::msg::Pose waypoint = start_pose;

  // Net doc ben trai: di tu duoi len tren
  waypoint.position.x =
    start_pose.position.x + letter_height;

  waypoints.push_back(waypoint);

  // Nang dau cong tac len sau khi ve xong net thu nhat
  waypoint.position.z =
    start_pose.position.z + lift_height;

  waypoints.push_back(waypoint);

  // Di chuyen tren cao den diem duoi ben phai
  waypoint.position.x = start_pose.position.x;
  waypoint.position.y =
    start_pose.position.y + letter_width;

  waypoints.push_back(waypoint);

  // Ha dau cong tac xuong mat phang ve
  waypoint.position.z = start_pose.position.z;

  waypoints.push_back(waypoint);

  // Net doc ben phai: di tu duoi len tren
  waypoint.position.x =
 
   start_pose.position.x + letter_height;

  waypoints.push_back(waypoint);

  // Nang dau cong tac len sau net thu hai
  waypoint.position.z =
    start_pose.position.z + lift_height;

  waypoints.push_back(waypoint);

  // Di chuyen tren cao den diem giua ben trai
  waypoint.position.x =
    start_pose.position.x + letter_height / 2.0;

  waypoint.position.y = start_pose.position.y;

  waypoints.push_back(waypoint);

  // Ha dau cong tac xuong mat phang ve
  waypoint.position.z = start_pose.position.z;

  waypoints.push_back(waypoint);

  // Ve net ngang tu trai sang phai
  waypoint.position.y =
    start_pose.position.y + letter_width;

  waypoints.push_back(waypoint);

  // Nang dau cong tac len sau khi ve xong
  waypoint.position.z =
    start_pose.position.z + lift_height;

  waypoints.push_back(waypoint);

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
      "Bat dau ve chu H");

    auto execute_result = move_group.execute(trajectory);

    if (execute_result == moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_INFO(
        node->get_logger(),
        "Da ve xong chu H");
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
