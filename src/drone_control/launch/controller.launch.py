from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch.actions
package_name = 'drone_control'

def generate_launch_description():

    swarm_waypoints = LaunchConfiguration('swarm_waypoints', default='-')
    formation_arg = LaunchConfiguration('formation_arg', default='-')
    formation_spacing = LaunchConfiguration('formation_spacing', default='-')
    swarm_size = LaunchConfiguration('swarm_size', default='-')
    drone_ns = LaunchConfiguration('drone_ns', default='-')
    
    #gps_name = ''.join([drone_ns,'_gps_subscriber'])#f'{drone_ns}_gps_subscriber'
    drone_ns_launch_arg = DeclareLaunchArgument(
        'drone_ns',
        default_value='drone_ns'
    )

    drone_spawn_x = LaunchConfiguration('drone_spawn_x')
    drone_spawn_x_launch_arg = DeclareLaunchArgument(
        'drone_spawn_x',
        default_value='0.0'
    )

    drone_spawn_y = LaunchConfiguration('drone_spawn_y')
    drone_spawn_y_launch_arg = DeclareLaunchArgument(
        'drone_spawn_y',
        default_value='0.0'
    )

    drone_spawn_z = LaunchConfiguration('drone_spawn_z')
    drone_spawn_z_launch_arg = DeclareLaunchArgument(
        'drone_spawn_z',
        default_value='0.0'
    )

    drone_spawn_yaw = LaunchConfiguration('drone_spawn_yaw')
    drone_spawn_yaw_launch_arg = DeclareLaunchArgument(
        'drone_spawn_yaw',
        default_value='0.0'
    )

 
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # ld = LaunchDescription()

    # gps_sub_node = Node(
    #     package=package_name,
    #     executable='gps_subscriber',
    #     # name = 'gps_subscriber', #NOTE: Adding the name created a duplicate node and would interfere with the namespace application. The service wouldnt work.
    #     namespace= drone_ns, #NOTE: Service not functioning with nsamespace prefix when a name is applied.
    #     parameters=[{'use_sim_time': use_sim_time}],
    #     output='screen',
    #     remappings=[('/gps', '/drone_1/gps'),]
    #     )
    
    # gps_sub_node_2 = Node(
    #     package=package_name,
    #     executable='gps_subscriber',
    #     # name = 'gps_subscriber', #NOTE: Adding the name created a duplicate node and would interfere with the namespace application. The service wouldnt work.
    #     namespace= 'drone_2', #NOTE: Service not functioning with nsamespace prefix when a name is applied.
    #     parameters=[{'use_sim_time': use_sim_time}],
    #     output='screen',
    #     remappings=[('/gps', '/drone_2/gps'),]
    #     )
    # gps_sub_node_3 = Node(
    #     package=package_name,
    #     executable='gps_subscriber',
    #     # name = 'gps_subscriber', #NOTE: Adding the name created a duplicate node and would interfere with the namespace application. The service wouldnt work.
    #     namespace= 'drone_3', #NOTE: Service not functioning with nsamespace prefix when a name is applied.
    #     parameters=[{'use_sim_time': use_sim_time}],
    #     output='screen',
    #     remappings=[('/gps', '/drone_3/gps'),]
    #     )
    
    # clock_sub_node = Node(
    #     package=package_name,
    #     executable='clock_subscriber',
    #     # name='clock_subscriber',
    #     parameters=[{'use_sim_time': use_sim_time}],
    #     output='screen')

    # ld.add_action(DeclareLaunchArgument(
    #     'use_sim_time',
    #     default_value='true',
    #     description='Use simulation (Gazebo) clock if true'),)
    
    # ld.add_action(gps_sub_node)
    # ld.add_action(gps_sub_node_2)
    # ld.add_action(gps_sub_node_3)
    # ld.add_action(clock_sub_node)

    # return ld

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        # Node(
        #     package=package_name,
        #     executable='state_publisher',
        #     name='state_publisher',
        #     output='screen'),

        #Node for sending out communication data
        Node(
            package=package_name,
            executable='drone_com_out',
            # name = 'gps_subscriber',
            namespace= drone_ns, 
            parameters=[
                {'use_sim_time': use_sim_time,
                'drone_ns': drone_ns,
                } #pass spawn location/orientation to gps
                ],

            output='screen',
 
            ),
        
        #Node for receiving communication data
        Node(
            package=package_name,
            executable='drone_com_in',
            # name = 'gps_subscriber',
            namespace= drone_ns, 
            parameters=[
                {'use_sim_time': use_sim_time,
                'drone_ns': drone_ns,
                'swarm_size': swarm_size,
                } #pass spawn location/orientation to gps
                ],

            output='screen',
 
            ),
        Node(
            package=package_name,
            executable='gps_subscriber',
            # name = 'gps_subscriber',
            namespace= drone_ns,
            parameters=[
                {'use_sim_time': use_sim_time,
                'drone_spawn_x': drone_spawn_x,
                'drone_spawn_y': drone_spawn_y,
                'drone_spawn_z': drone_spawn_z,
                'drone_spawn_yaw': drone_spawn_yaw} #pass spawn location/orientation to gps
                ],

            output='screen',
            # remappings=[
            #     ('/gps', '/drone_1/gps'),
            # ]

            ),

        Node(
            package=package_name,
            executable='imu_subscriber',
            namespace= drone_ns,
            # name='imu_subscriber',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),

        Node(
            package=package_name,
            executable='clock_subscriber',
            namespace= drone_ns,
            # name='clock_subscriber',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
            # remappings=[
            #     ('/imu_main_body/out', '/drone_1/imu_main_body/out'),
            # ]
            ),

        Node(
            package=package_name,
            executable='behavioral_controller',
            # name='behavioral_controller',
            namespace= drone_ns,
            parameters=[{
                'use_sim_time': use_sim_time,
                'swarm_waypoints': swarm_waypoints,
                'formation_arg': formation_arg,
                'formation_spacing': formation_spacing,
                'drone_ns': drone_ns,
                'swarm_size': swarm_size,
                }],
            output='screen'),

        # Node(
        #     package=package_name,
        #     executable='front_pt_cloud_subscriber',
        #     name='front_pt_cloud_subscriber',
        #     parameters=[{'use_sim_time': use_sim_time}],
        #     output='screen'),
        
        Node(
            package=package_name,
            executable='pos_controller',
            # name='pos_controller',
            namespace= drone_ns,
            parameters=[{
                'use_sim_time': use_sim_time,
                'drone_ns': drone_ns,
                }],
            output='screen'),

        Node(
            package=package_name,
            executable='attitude_controller',
            # name='attitude_controller',
            namespace= drone_ns,
            parameters=[{
                'use_sim_time': use_sim_time,
                'drone_ns': drone_ns,
                }],
            output='screen'),

        Node(
            package=package_name,
            executable='drone_controller_conductor',
            # name='drone_controller_conductor',
            namespace= drone_ns,
            parameters=[{
                'use_sim_time': use_sim_time,
                'drone_ns': drone_ns,
                }],
            output='screen'),

        # Node(
        #     package=package_name,
        #     executable='mpc_controller',
        #     # name='mpc_controller',
        #     namespace= drone_ns,
        #     parameters=[{'use_sim_time': use_sim_time}],
        #     output='screen'),
            
        Node(
            package=package_name,
            executable='velocity_calc_service',
            # name='velocity_calc_service',
            namespace= drone_ns,
            parameters=[{
                'use_sim_time': use_sim_time,
                'drone_ns': drone_ns,
                }],
            output='screen'),
    ])