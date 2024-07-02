from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch.actions
package_name = 'drone_control'

def generate_launch_description():
    
    formation_arg = LaunchConfiguration('formation_arg', default='-')
    formation_spacing = LaunchConfiguration('formation_spacing', default='-')
    swarm_waypoints = LaunchConfiguration('swarm_waypoints', default='-')
    swarm_size = LaunchConfiguration('swarm_size', default='-')
    drone_ns = LaunchConfiguration('drone_ns', default='-')
    
    #gps_name = ''.join([drone_ns,'_gps_subscriber'])#f'{drone_ns}_gps_subscriber'
    drone_ns_launch_arg = DeclareLaunchArgument(
        'drone_ns',
        default_value='drone_ns'
    )


 
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),



        Node(
            package=package_name,
            executable='cmd_fwd_publisher',
            # name='behavioral_controller',
            namespace= drone_ns,
            parameters=[{
                'use_sim_time': use_sim_time,
                'drone_ns': drone_ns,
                'swarm_size': swarm_size,
                'formation_arg': formation_arg,
                'formation_spacing': formation_spacing,
                'swarm_waypoints': swarm_waypoints,

                }],
            output='screen'),


    ])