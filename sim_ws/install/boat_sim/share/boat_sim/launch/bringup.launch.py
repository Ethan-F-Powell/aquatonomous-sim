# launch/bringup.launch.py
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg    = get_package_share_directory('boat_sim')
    models = os.path.join(pkg, 'models')
    worlds = os.path.join(pkg, 'worlds')
    world  = os.path.join(worlds, 'flat_water.sdf')

    return LaunchDescription([
        # Make models and worlds discoverable
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', f'{models}:{worlds}'),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH',     f'{models}:{worlds}'),
        
        ExecuteProcess(cmd=['ign','gazebo','-r','-v','4', world], output='screen'),
        # Stable GUI on WSLg (software rendering)
        SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'),

        # Launch Gazebo Fortress GUI
        ExecuteProcess(
            cmd=['ign', 'gazebo', '-r', world],
            output='screen'
        ),

        # Bridge ROS -> Ignition (publish Float64 from ROS into sim)
        Node(
            package='ros_ign_bridge',
            executable='parameter_bridge',
            arguments=[
                '/model/nd_boat/joint/left_prop_joint/cmd_thrust@std_msgs/msg/Float64]ignition.msgs.Double',
                '/model/nd_boat/joint/right_prop_joint/cmd_thrust@std_msgs/msg/Float64]ignition.msgs.Double',
            ],
            output='screen'
        ),
    ])
