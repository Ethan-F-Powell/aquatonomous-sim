# launch/bringup.launch.py
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg    = get_package_share_directory('boat_sim')
    models = os.path.join(pkg, 'models')
    worlds = os.path.join(pkg, 'worlds')
    world  = os.path.join(worlds, 'flat_water.sdf')

    return LaunchDescription([
        # Env so the sim finds assets and all terminals share the same bus
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', f'{models}:{worlds}'),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH',     f'{models}:{worlds}'),
        SetEnvironmentVariable('IGN_TRANSPORT_PARTITION',        'aquatonomous'),
        SetEnvironmentVariable('IGN_TRANSPORT_TOPIC_PARTITION',  'aquatonomous'),
        SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'),
        SetEnvironmentVariable('QT_QUICK_BACKEND', 'software'),
        SetEnvironmentVariable('QSG_RENDER_LOOP', 'basic'),

        # Kill stale processes, then start ONE combined server+GUI.
        # IMPORTANT: Do NOT start a second ign anywhere else.
        ExecuteProcess(
            cmd=[
                'bash', '-lc',
                'pkill -f ign-gazebo || true; '
                'exec ign gazebo -v 4 "{}"'.format(world)
            ],
            output='screen',
        ),
    ])
