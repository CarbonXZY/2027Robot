# chassis.launch.py
# 启动：robot_state_publisher + ros2_control_node + mecanum_drive_controller。
# 运行：ros2 launch chassis chassis.launch.py
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    chassis_dir = get_package_share_directory('chassis')

    urdf_path = os.path.join(chassis_dir, 'urdf', 'chassis.urdf')
    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    controller_config = os.path.join(chassis_dir, 'config', 'chassis_controller.yaml')

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description, 'use_sim_time': False}],
        ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[{'robot_description': robot_description, 'update_rate': 1000}],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'mecanum_drive_controller',
                '--controller-type', 'mecanum_drive_controller/MecanumDriveController',
                '--param-file', controller_config,
            ],
            output='screen',
        ),
    ])
