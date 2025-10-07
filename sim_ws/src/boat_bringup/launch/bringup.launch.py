
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    bridge_cfg = os.path.join(get_package_share_directory('boat_bringup'), 'config', 'bridge.yaml')
    world = os.path.join(get_package_share_directory('boat_bringup'), 'config', 'test_world.sdf')
    rviz_cfg = os.path.join(get_package_share_directory('boat_bringup'), 'config', 'rviz.rviz')

    gz = Node(
        package='ros_gz_sim',
        executable='gz_sim',
        arguments=['-r', world],
        output='screen'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['@' + bridge_cfg],
        output='screen'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],
        output='screen'
    )

    return LaunchDescription([gz, bridge, rviz])
