from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

# from drone_control.drone_control.constants import (
#     DRONE_SPAWN,
# )

name_pkg = "drone_gazebo"
pkg_share = FindPackageShare(package=name_pkg).find(name_pkg)

world_file_name = 'base.world'
world_path = os.path.join(pkg_share, 'worlds', world_file_name)

param_file = os.path.join(pkg_share, 'config', 'params.yaml')
 
# def gen_robot_list(number_of_robots):

#     robots = []

#     for i in range(number_of_robots):
#         robot_name = "drone"+str(i)
#         x_pos = float(i)
#         robots.append({'name': robot_name, 'x_pose': x_pos, 'y_pose': 0.0, 'z_pose': 1.0})

#     return robots 

def generate_launch_description():    
    # robot_description_topic_name = "robot_description"
    # spawn_x_val = '0.0'
    # spawn_y_val = '0.0'
    # spawn_z_val = '1.0'
    # spawn_yaw_val = '0.0'

    # drone_ns = LaunchConfiguration('drone_ns', default='-')
    # drone_ns_launch_arg = DeclareLaunchArgument(
    #     'drone_ns',
    #     default_value='drone_ns'
    # )
    # drone_spawn_x = LaunchConfiguration('drone_spawn_x', default='-')
    # drone_spawn_x_launch_arg = DeclareLaunchArgument(
    #     'drone_spawn_x',
    #     default_value='0.0'
    # )

    # drone_spawn_y = LaunchConfiguration('drone_spawn_y', default='-')
    # drone_spawn_y_launch_arg = DeclareLaunchArgument(
    #     'drone_spawn_y',
    #     default_value='0.0'
    # )

    # drone_spawn_z = LaunchConfiguration('drone_spawn_z', default='-')
    # drone_spawn_z_launch_arg = DeclareLaunchArgument(
    #     'drone_spawn_z',
    #     default_value='0.0'
    # )

    # drone_spawn_yaw = LaunchConfiguration('drone_spawn_yaw', default='-')
    # drone_spawn_yaw_launch_arg = DeclareLaunchArgument(
    #     'drone_spawn_yaw',
    #     default_value='0.0'
    # )

    return LaunchDescription([
        ExecuteProcess(
            #cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', 'worlds/forest.world'], # forest world fails to load properly. maybe check for a positioning reference requirement.
            cmd=[
                'gazebo', 
                '--verbose', 
                '-s', 'libgazebo_ros_init.so',
                '-s', 'libgazebo_ros_factory.so', 
                world_path,
                '--ros-args', '--params-file', param_file,
                ],
            output='screen'
        ),
        # Node(
        #     package='gazebo_ros',
        #     executable='spawn_entity.py',
        #     output='screen',
        #     arguments=[
        #         '-entity', drone_ns,#matrice300rtk',
        #         '-topic', robot_description_topic_name,
        #         "-robot_namespace", drone_ns,
        #         '-x', drone_spawn_x,
        #             '-y', drone_spawn_y,
        #             '-z', drone_spawn_z,
        #             '-Y', drone_spawn_yaw
        #             ],                    
        # ),

        # Node(
        #     package='gazebo_ros',
        #     executable='spawn_entity.py',
        #     output='screen',
        #     arguments=[
        #         '-entity', 'drone_2',#matrice300rtk',
        #         '-topic', robot_description_topic_name,
        #         "-robot_namespace","drone_2",
        #         '-x', spawn_x_val,
        #             '-y', '1.0',
        #             '-z', spawn_z_val,
        #             '-Y', spawn_yaw_val
        #             ],                    
        # ),

        # Node(
        #     package='gazebo_ros',
        #     executable='spawn_entity.py',
        #     output='screen',
        #     arguments=[
        #         '-entity', 'drone_3',#matrice300rtk',
        #         '-topic', robot_description_topic_name,
        #         "-robot_namespace","drone_3",
        #         '-x', spawn_x_val,
        #             '-y', '2.0',
        #             '-z', spawn_z_val,
        #             '-Y', spawn_yaw_val
        #             ],                    
        # ),

    ])

