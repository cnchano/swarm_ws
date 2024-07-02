from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory

import launch.actions
import launch_ros.actions

DESCRIPTION_PKG = 'drone_description'

def generate_launch_description():

    return LaunchDescription([
        launch_ros.actions.Node(
            package = DESCRIPTION_PKG,
            executable = 'spawn_drone_exe',
            output='screen',
            arguments=[
                '--drone_urdf', launch.substitutions.LaunchConfiguration('drone_urdf'),
                '--drone_name', launch.substitutions.LaunchConfiguration('drone_name'),
                '--drone_ns', launch.substitutions.LaunchConfiguration('drone_ns'),
                '-drone_spawn_x', launch.substitutions.LaunchConfiguration('drone_spawn_x'),
                '-drone_spawn_y', launch.substitutions.LaunchConfiguration('drone_spawn_y'),
                '-drone_spawn_z', launch.substitutions.LaunchConfiguration('drone_spawn_z')]),
    ])