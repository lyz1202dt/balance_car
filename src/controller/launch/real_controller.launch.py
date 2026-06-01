import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


def _load_real_robot_description():
    xacro_path = os.path.join(get_package_share_directory("controller"), "urdf", "real_car.urdf.xacro")
    return xacro.process_file(xacro_path).toprettyxml(indent="  ")


def generate_launch_description():
    robot_description = {"robot_description": _load_real_robot_description()}
    controller_yaml = os.path.join(get_package_share_directory("controller"), "config", "real_controller.yaml")

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_yaml, robot_description],
        output="screen",
    )

    lqr_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["lqr_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    return LaunchDescription(
        [
            ros2_control_node,
            lqr_controller_spawner,
        ]
    )
