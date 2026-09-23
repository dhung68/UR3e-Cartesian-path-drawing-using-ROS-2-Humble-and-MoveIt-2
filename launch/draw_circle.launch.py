import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    ur_simulation_launch = os.path.join(
        get_package_share_directory("ur_simulation_gz"),
        "launch",
        "ur_sim_moveit.launch.py",
    )

    start_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ur_simulation_launch),
        launch_arguments={
            "ur_type": "ur3e",
        }.items(),
    )

    start_circle_node = TimerAction(
        period=30.0,
        actions=[
            Node(
                package="ur3e_letter_drawer",
                executable="circle_drawer_node",
                name="circle_drawer",
                output="screen",
            )
        ],
    )

    return LaunchDescription([
        start_simulation,
        start_circle_node,
    ])