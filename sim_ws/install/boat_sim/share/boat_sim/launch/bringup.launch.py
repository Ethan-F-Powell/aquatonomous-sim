from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg    = get_package_share_directory("boat_sim")
    models = os.path.join(pkg, "models")
    worlds = os.path.join(pkg, "worlds")
    world  = os.path.join(worlds, "flat_water.sdf")

    # Same resource path as your manual command
    resource_path = f"{models}:{worlds}"

    return LaunchDescription([
        # Match your manual `export` calls
        SetEnvironmentVariable(
            name="IGN_GAZEBO_RESOURCE_PATH",
            value=resource_path
        ),
        SetEnvironmentVariable(
            name="GZ_SIM_RESOURCE_PATH",
            value=resource_path
        ),
        SetEnvironmentVariable(
            name="IGN_TRANSPORT_PARTITION",
            value="aquatonomous"
        ),
        SetEnvironmentVariable(
            name="IGN_TRANSPORT_TOPIC_PARTITION",
            value="aquatonomous"
        ),

        # Software rendering that made the GUI stable
        SetEnvironmentVariable(
            name="LIBGL_ALWAYS_SOFTWARE",
            value="1"
        ),
        SetEnvironmentVariable(
            name="MESA_LOADER_DRIVER_OVERRIDE",
            value="llvmpipe"
        ),
        SetEnvironmentVariable(
            name="GALLIUM_DRIVER",
            value="llvmpipe"
        ),

        # Start Ignition like your working manual call
        ExecuteProcess(
            cmd=["ign", "gazebo", "-v", "2", world],
            output="screen",
        ),
    ])
