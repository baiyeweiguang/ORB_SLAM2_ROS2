import os
import sys
from ament_index_python.packages import get_package_share_directory
sys.path.append(os.path.join(get_package_share_directory('orb_slam2_ros2'), 'launch'))


def generate_launch_description():
    from launch_ros.actions import Node
    from launch import LaunchDescription

    node_params = os.path.join(
    get_package_share_directory('orb_slam2_ros2'), 'config', 'mono.yaml')

    mono_node = Node(
        package='orb_slam2_ros2',
        executable='mono_node',
        emulate_tty=True,
        output='both',
        parameters=[node_params],
    )

    return LaunchDescription([
        mono_node
    ])