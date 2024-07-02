import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, TextSubstitution
from launch_ros.actions import Node
import xacro

from launch.launch_description_sources import PythonLaunchDescriptionSource

package_name = 'drone_description'

def generate_launch_description():
    drone_ns = LaunchConfiguration('drone_ns', default='-')

    drone_description_topic_name = LaunchConfiguration('drone_description_topic_name', default='-')
    
    urdf_file_name = 'drone.xacro'
    #ref_station_urdf_file_name = 'reference.station.xacro' #Idea for creating gps reference, by creating seperate model. But better to duplicate drone and not command it to move
    urdf = os.path.join(
        get_package_share_directory(package_name), "urdf", urdf_file_name
    )


    # robot_description_config = xacro.process_file(urdf, mappings={'robot_name_arg': 'drone_1'}) #NOTE: Remapping doesn't work
    robot_description_config = xacro.process_file(urdf)
    robot_desc = robot_description_config.toxml()



    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name= drone_description_topic_name,
            #namespace='drone111',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, drone_description_topic_name: robot_desc}], #Command([urdf, 'robot_name:=', entity_name])
            #arguments=[urdf]
            ),
        # Node(
        #     package='robot_state_publisher',
        #     executable='robot_state_publisher',
        #     name='robot_state_publisher',
        #     #namespace='drone333',
        #     output='screen',
        #     parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}], #Command([urdf, 'robot_name:=', entity_name])
        #     #arguments=['drone111':='drone333'],
        #     ),
    ])
    #/////////////////////////////////////////////