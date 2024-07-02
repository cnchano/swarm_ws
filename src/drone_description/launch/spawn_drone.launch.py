
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration



name_pkg = "drone_description"
pkg_share = FindPackageShare(package=name_pkg).find(name_pkg)


 

def generate_launch_description():    
    # robot_description_topic_name = "robot_description" 
    drone_description_topic_name = LaunchConfiguration('drone_description_topic_name', default='-')

    drone_ns = LaunchConfiguration('drone_ns', default='-')
    drone_spawn_x = LaunchConfiguration('drone_spawn_x', default='-')
    drone_spawn_y = LaunchConfiguration('drone_spawn_y', default='-')
    drone_spawn_z = LaunchConfiguration('drone_spawn_z', default='-')
    drone_spawn_yaw = LaunchConfiguration('drone_spawn_yaw', default='-')

    return LaunchDescription([

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            output='screen',
            arguments=[
                '-entity', drone_ns,
                '-topic', drone_description_topic_name, #robot_description_topic_name,
                "-robot_namespace", drone_ns,
                '-x', drone_spawn_x,
                '-y', drone_spawn_y,
                '-z', drone_spawn_z,
                '-Y', drone_spawn_yaw
                ],                    
        ),

    ])

 