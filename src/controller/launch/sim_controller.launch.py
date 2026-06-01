import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_sim_robot_description():
    urdf_path = os.path.join(get_package_share_directory("car"), "model", "car.urdf")
    with open(urdf_path, "r", encoding="utf-8") as urdf_file:
        robot_description = urdf_file.read()

    robot_description = robot_description.replace(
        "<plugin>mujoco_ros2_control/MujocoSystem</plugin>",
        "<plugin>controller/MujocoSimSystem</plugin>",
    )
    robot_description = robot_description.replace(
        "</hardware>",
        "            <param name=\"imu_sensor_name\">imu</param>\n"
        "            <param name=\"imu_topic\">/imu_imu_sensor/imu</param>\n"
        "        </hardware>",
        1,
    )
    return robot_description


def generate_launch_description():
    show_gui = LaunchConfiguration("show_gui")
    simulation_frequency = LaunchConfiguration("simulation_frequency")
    realtime_factor = LaunchConfiguration("realtime_factor")

    robot_description = {"robot_description": _load_sim_robot_description()}
    controller_yaml = os.path.join(get_package_share_directory("controller"), "config", "sim_controller.yaml")
    mujoco_model_path = os.path.join(get_package_share_directory("car"), "model", "scene.xml")

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description, {"use_sim_time": True}],
        output="screen",
    )

    mujoco = Node(
        package="mujoco_ros2_control",
        executable="mujoco_ros2_control",
        parameters=[
            robot_description,
            controller_yaml,
            {"simulation_frequency": simulation_frequency},
            {"realtime_factor": realtime_factor},
            {"robot_model_path": mujoco_model_path},
            {"show_gui": show_gui},
        ],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
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

    load_controllers = RegisterEventHandler(
        OnProcessStart(
            target_action=mujoco,
            on_start=[
                joint_state_broadcaster_spawner,
                car_controller_spawner,
            ],
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("show_gui", default_value="true"),
            DeclareLaunchArgument("simulation_frequency", default_value="500.0"),
            DeclareLaunchArgument("realtime_factor", default_value="1.0"),
            robot_state_publisher,
            mujoco,
            load_controllers,
        ]
    )
