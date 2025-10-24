# launch/bringup.launch.py
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('boat_sim')
    models = os.path.join(pkg, 'models')
    worlds = os.path.join(pkg, 'worlds')
    world  = os.path.join(worlds, 'flat_water.sdf')

    return LaunchDescription([
        # Make models and worlds discoverable
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', f'{models}:{worlds}'),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH',     f'{models}:{worlds}'),

        # Stable GUI on WSLg (software rendering)
        SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'),

        # Launch Gazebo Fortress GUI
        ExecuteProcess(
            cmd=['ign', 'gazebo', '-r', world],
            output='screen'
        ),

        # Bridge (use ros_ign_bridge; it may print a notice and redirect to ros_gz_bridge)
        Node(
            package='ros_ign_bridge',
            executable='parameter_bridge',
            arguments=[os.path.join(pkg, 'config', 'bridge.yaml')],
            output='screen'
        ),
    ])
