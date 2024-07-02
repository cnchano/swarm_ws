import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

package_name = 'drone_rviz'

def generate_launch_description():
    rviz_config_file_name = 'drone.rviz'
    rviz_config = os.path.join(
        get_package_share_directory(package_name), "config", rviz_config_file_name
    )

    return LaunchDescription([
        Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            # arguments=['-d' + rviz_config]
        ),
    ])