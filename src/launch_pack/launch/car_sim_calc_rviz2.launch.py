from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
import os

def generate_launch_description():

    simulate_env_launch_scripe="car_mujoco_sim.py"

    urdf_path = os.path.join(
        get_package_share_directory("car"),
        "model", "car.urdf"
    )
    # 读取URDF内容
    with open(urdf_path, 'r') as inf:
        robot_desc = inf.read()

    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_desc}]
    )

    leg_calc = Node(
        package="leg_calc",
        executable="leg_calc"
    )

    rviz2_config_path=os.path.join(
        get_package_share_directory("launch_pack"),
        "rviz", "display_config.rviz"
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz2_config_path]  # 可选，指定rviz配置文件
    )
    
    sim_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('launch_pack'), 'launch', simulate_env_launch_scripe)]))
    
    return LaunchDescription([robot_state_pub,  leg_calc , rviz2 ,sim_launch])
