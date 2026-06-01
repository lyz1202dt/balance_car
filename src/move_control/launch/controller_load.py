from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    urdf_path = os.path.join(
        get_package_share_directory("car"),
        "urdf", "car.urdf"
    )

    controller_yaml=os.path.join(
        get_package_share_directory("move_control"),
        "config", "ros2_controller.yaml"
    )

    # 读取URDF内容
    with open(urdf_path, 'r') as inf:
        robot_desc = inf.read()

    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_desc}]
    )
    
    ros2_control_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_yaml,
                    {"robot_description": robot_desc},
        ],
        output="screen"
    )

    # 使用 spawner 启动具体的 controller（这里的名字需要与 ros2_controller.yaml 中的键一致）
    spawner_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["lqr_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

   
    return LaunchDescription([
        robot_state_pub,
        ros2_control_manager,
        spawner_node,
    ])
