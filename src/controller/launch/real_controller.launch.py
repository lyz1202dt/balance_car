import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def _load_real_robot_description():
    urdf_path = os.path.join(get_package_share_directory("car"), "model", "car.urdf")
    with open(urdf_path, "r", encoding="utf-8") as urdf_file:
        robot_description = urdf_file.read()

    robot_description = robot_description.replace(
        "<plugin>mujoco_ros2_control/MujocoSystem</plugin>",
        "<plugin>controller/RealHardware</plugin>",
    )
    robot_description = robot_description.replace(
        "</hardware>",
        "            <param name=\"imu_sensor_name\">imu</param>\n"
        "        </hardware>",
        1,
    )
    return robot_description


def generate_launch_description():
    robot_description = {"robot_description": _load_real_robot_description()}
    controller_yaml = os.path.join(get_package_share_directory("controller"), "config", "real_controller.yaml")

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_yaml, robot_description],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    car_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["car_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    return LaunchDescription(
        [
            robot_state_publisher,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            car_controller_spawner,
        ]
    )
