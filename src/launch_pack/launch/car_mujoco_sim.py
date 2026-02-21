from launch.event_handlers import OnExecutionComplete, OnProcessStart, OnProcessExit
from launch_ros.actions import Node
import os
from ament_index_python import get_package_share_directory 
from ament_index_python import get_package_prefix
from launch.actions import ExecuteProcess
from launch.substitutions import FindExecutable
from launch import LaunchDescription
from launch.actions import (
    LogInfo,
    RegisterEventHandler
)


def generate_launch_description():
    car_pack_path=get_package_share_directory("car")
    launch_pack_path=get_package_share_directory("launch_pack")

    namespace=""
    mjcf_file=os.path.join(car_pack_path,"model","scene.xml")
    urdf_file=os.path.join(car_pack_path,"model","car.urdf")
    controller_config_file=os.path.join(launch_pack_path,"config","ros2_controller.yaml")

    # 读取URDF文件内容并作为字符串存储
    with open(urdf_file, 'r') as inf:
        robot_desc = inf.read()

    mujoco = Node(
        package="mujoco_ros2_control",
        executable="mujoco_ros2_control",
        namespace=namespace,
        parameters=[
            {"robot_description":robot_desc},
            controller_config_file,
            {"simulation_frequency": 500.0},
            {"realtime_factor": 1.0},
            {"robot_model_path": mjcf_file},
            {"show_gui": True},
        ],
        remappings=[
            ('/controller_manager/robot_description', '/robot_description'),
        ]
    )

    joint_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["car_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    load_controllers = RegisterEventHandler(    #仿真环境一旦加载完成，开始加载控制器
        OnProcessStart(
            target_action=mujoco,
            on_start=[
                LogInfo(msg="Starting control"),
                joint_controller,
            ],
        )
    )

    
    return LaunchDescription([
        mujoco,
        load_controllers
        ])