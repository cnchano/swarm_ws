import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.conditions import IfCondition
import launch.actions
# from .spawn_constants import (
#     DRONE_SPAWN,
# ) #NOTE: Could not access file since python could not find the parent package


DESCRIPTION_PKG = 'drone_description'
CONTROLLER_PKG = 'drone_control'
RVIZ_PKG = 'drone_rviz'
GAZEBO_PKG = 'drone_gazebo'
INTERCOM_PKG = 'drone_intercom'


#NOTE: FORMATION CHANGES AT DESIGNATED WAYPOINTS, BUT IS NOT REQUIRED TO CHANGE FORMATION OVER MANY WAYPOINTS

#SINGLE DRONE SURVEY
#SWARM_WAYPOINTS = '[77.15, 75.00, 15.00, 77.15, 375.00, 15.00, 100.00, 375.00, 15.00, 100.00, 75.00, 15.00, 122.85, 75.00, 15.00, 122.85, 375.00, 15.00, 145.70, 375.00, 15.00, 145.70, 75.00, 15.00, 168.55, 75.00, 15.00, 168.55, 375.00, 15.00, 191.40, 375.00, 15.00, 191.40, 75.00, 15.00, 214.25, 75.00, 15.00, 214.25, 375.00, 15.00, 237.10, 375.00, 15.00, 237.10, 75.00, 15.00, 259.95, 75.00, 15.00, 259.95, 375.00, 15.00, 282.80, 375.00, 15.00, 282.80, 75.00, 15.00, 305.65, 75.00, 15.00, 305.65, 375.00, 15.00, 328.50, 375.00, 15.00, 328.50, 75.00, 15.00, 351.35, 75.00, 15.00, 351.35, 375.00, 15.00, 374.20, 375.00, 15.00, 374.20, 75.00, 15.00, 0.00, 0.00, 15.00]'
#DRONE FORMATION CHANGE TEST
SWARM_WAYPOINTS ='[20.0, 0.0, 15.0, 20.0, 20.0, 15.0, 0.0, 20.0, 15.0, -20.0, 20.0, 15.0, -20.0, 0.0, 15.0, -20.0, -20.0, 15.0, 0.0, -20.0, 15.0, 20.0, -20.0, 15.0, 20.0, 0.0, 15.0, 0.0, 0.0, 15.0]' #'[ 20.0, 20.0, 15.0, 40.0, 0.0, 15.0, 60.0, -20.0, 15.0, 80.0, 0.0, 15.0, 100.0, 0.0, 15.0]'  #'[ 20.0, 20.0, 15.0, 40.0, 0.0, 15.0, 60.0, -20.0, 15.0, 80.0, 0.0, 15.0, 100.0, 0.0, 15.0]' #Goal point of the swarm NOTE: THE WHOLE ARRAY MUST BE WRITTEN AS STRING TO BE PASSED AS AN ARRAY
#MULTI-DRONE SURVEY
# SWARM_WAYPOINTS = '[ 20.0, 0.0, 20.0, 100.0, 0.0, 20.0, 100.0, 75.0, 20.0, 100.0, 375.0, 20.0, 168.55, 375.0, 20.0, 168.55, 75.0, 20.0 , 237.1, 75.0, 20.0, 237.1, 375.0, 20.0, 305.65, 375.0, 20.0, 305.65, 75.0, 20.0, 374.2, 75.0, 20.0, 374.2, 375.0, 20.0, 0.0, 0.0, 20.0]'

#COLLISION AVOIDANCE TEST
FORMATION_ARG = '[ single_file, circle, single_file, single_row, single_file, circle, single_row, circle, single_row, single_file]' #'[single_file, single_file, single_row, single_row, circle]' # #'[single_file, single_file, single_row, single_row, circle]' #[formation name] NOTE: Parameters can only be a single type  
FORMATION_SPACING = '[8, 8, 8, 8, 8, 8, 8, 8, 8, 8]' #'[8, 8, 8, 8, 15]' #'[ 8, 8, 8, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 8]' #'[8, 8, 8, 8, 15]'

# SURVEY MISSION
# FORMATION_ARG = '[' + ', '.join(['circle'] * 2) + ',' + ', '.join(['single_row'] * 10) + ', circle' + ']' #27 for single drone
# FORMATION_SPACING = '[' + ', '.join(['8'] * 2) + ',' + ', '.join(['22'] * 11) + ', 8' + ']' #27 for single drone

SWARM_SIZE = '3' #number of drones in swarm
DRONE_SPAWN = {
    'drone_1_pos': ['-6.0', '-10.0', '1.0', '0.0'], #x, y, z [meters] , yaw 

    'drone_2_pos': ['-6.0', '10.0', '1.0', '0.0'], #x, y, z [meters] , yaw 

    'drone_3_pos': ['3.0', '3.0', '1.0', '0.0'], #x, y, z [meters] , yaw 

    'drone_4_pos': ['2.0', '-2.0', '1.0', '0.0'], #x, y, z [meters] , yaw 

    'drone_5_pos': ['0.0', '0.0', '1.0', '0.0'], #x, y, z [meters] , yaw 
}
# DRONE_SPAWN = {
#     'drone_1_pos': ['-3.0', '-3.0', '1.0', '0.0'], #x, y, z [meters] , yaw 

#     'drone_2_pos': ['3.0', '3.0', '1.0', '0.0'], #x, y, z [meters] , yaw 

#     'drone_3_pos': ['-3.0', '3.0', '1.0', '0.0'], #x, y, z [meters] , yaw 

#     'drone_4_pos': ['2.0', '-2.0', '1.0', '0.0'], #x, y, z [meters] , yaw 

#     'drone_5_pos': ['0.0', '0.0', '1.0', '0.0'], #x, y, z [meters] , yaw 
# }

def gen_drone_spawn_list(swarm_size, drone_spawn):

    drone_spawn_data_dict = []

    for i in range(swarm_size):
        drone_id = int(i+1)
        drone_pos_array = drone_spawn['drone_'+ f'{drone_id}' + '_pos']
        drone_spawn_data_dict.append({'drone_ns': 'drone_'+ f'{drone_id}', 'drone_spawn_x': drone_pos_array[0], 'drone_spawn_y': drone_pos_array[1], 'drone_spawn_z': drone_pos_array[2]})
    
    return drone_spawn_data_dict


#DEFINE CONTROLLER PACKAGE
def genDroneControllerLaunchDesc(namespace, swarm_size, swarm_waypoints, formation_arg, formation_spacing,  init_pos):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(
                    CONTROLLER_PKG), "launch", "controller.launch.py"
            )
        ),

        launch_arguments={
            'drone_ns': namespace,
            'swarm_size': swarm_size,
            'swarm_waypoints': swarm_waypoints,
            'formation_arg': formation_arg,
            'formation_spacing': formation_spacing,
            'drone_spawn_x': init_pos[0],
            'drone_spawn_y': init_pos[1],
            'drone_spawn_z': init_pos[2],
            'drone_spawn_yaw': init_pos[3],
        }.items(),
    )

#DEFINE GATE NODE INTERCOM PACKAGE
def genDroneIntercomLaunchDesc(namespace, swarm_size, swarm_waypoints, formation_arg, formation_spacing, init_pos):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(
                    CONTROLLER_PKG), "launch", "cmd_fwd.launch.py"
            )
        ),

        launch_arguments={
            'drone_ns': namespace,
            'swarm_size': swarm_size,
            'swarm_waypoints': swarm_waypoints,
            'formation_arg': formation_arg,
            'formation_spacing': formation_spacing,
            'drone_spawn_x': init_pos[0],
            'drone_spawn_y': init_pos[1],
            'drone_spawn_z': init_pos[2],
            'drone_spawn_yaw': init_pos[3],
        }.items(),
    )

# DEFINE DRONE SPAWNER PACKAGE
def genDroneSpawnLaunchDesc(namespace, sdf_topic_name, init_pos):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(
                    DESCRIPTION_PKG), "launch", "spawn_drone.launch.py"
            )
        ),

        launch_arguments={
            'drone_ns': namespace,
            'drone_description_topic_name': sdf_topic_name,
            'drone_spawn_x': init_pos[0],
            'drone_spawn_y': init_pos[1],
            'drone_spawn_z': init_pos[2],
            'drone_spawn_yaw': init_pos[3],
        }.items(),
    )

#DEFINE MULTI-DRONE SPAWNER
def genDroneMultiSpawnLaunchDesc(swarm_size, drone_spawn):
    drone_spawn_data_dict = gen_drone_spawn_list(swarm_size, drone_spawn)
    
    urdf_file_name = 'drone.xacro'
    urdf = os.path.join(
        get_package_share_directory(DESCRIPTION_PKG), "urdf", urdf_file_name
    )

        # We create the list of spawn robots commands
    spawn_drone_cmds = []
    for drone_spawn_data_dict in drone_spawn_data_dict:
        spawn_drone_cmds.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory(DESCRIPTION_PKG), 
                        'launch',
                        'multi_spawn_drone.launch.py'
                    )
                ),

                    launch_arguments={
                        'drone_urdf': urdf,
                        'drone_spawn_x': TextSubstitution(text=str(drone_spawn_data_dict['drone_spawn_x'])),
                        'drone_spawn_y': TextSubstitution(text=str(drone_spawn_data_dict['drone_spawn_y'])),
                        'drone_spawn_z': TextSubstitution(text=str(drone_spawn_data_dict['drone_spawn_z'])),
                        'drone_name': drone_spawn_data_dict['drone_ns'],
                        'drone_ns': drone_spawn_data_dict['drone_ns']
                    }.items()
                ),
            )
    return spawn_drone_cmds

#PLACE DRONE DEFINITION PACKAGES TO SINGLE VARIABLE FOR LAUNCH DESCRIPTION
DRONE_DESCRIPTION = [
    #INTERCOM FOR GATE NODE (DRONE 1)
    genDroneIntercomLaunchDesc('drone_1', SWARM_SIZE, SWARM_WAYPOINTS, FORMATION_ARG, FORMATION_SPACING,  DRONE_SPAWN['drone_1_pos']),

    #DRONE 1

    genDroneControllerLaunchDesc('drone_1', SWARM_SIZE, SWARM_WAYPOINTS, FORMATION_ARG, FORMATION_SPACING, DRONE_SPAWN['drone_1_pos']),

    # genDroneSpawnLaunchDesc('drone_1', 'drone_1_robot_description', DRONE_SPAWN['drone_1_pos']),
    #DRONE 2
    #INTERCOM FOR GATE NODE (DRONE 2)
    genDroneIntercomLaunchDesc('drone_2', SWARM_SIZE, SWARM_WAYPOINTS, FORMATION_ARG, FORMATION_SPACING,  DRONE_SPAWN['drone_2_pos']),

    genDroneControllerLaunchDesc('drone_2', SWARM_SIZE, SWARM_WAYPOINTS, FORMATION_ARG, FORMATION_SPACING, DRONE_SPAWN['drone_2_pos']),

    # genDroneSpawnLaunchDesc('drone_2', 'drone_2_robot_description', DRONE_SPAWN['drone_2_pos']),
    #DRONE 3
    #INTERCOM FOR GATE NODE (DRONE 3)
    genDroneIntercomLaunchDesc('drone_3', SWARM_SIZE, SWARM_WAYPOINTS, FORMATION_ARG, FORMATION_SPACING,  DRONE_SPAWN['drone_3_pos']),

    genDroneControllerLaunchDesc('drone_3', SWARM_SIZE, SWARM_WAYPOINTS, FORMATION_ARG, FORMATION_SPACING, DRONE_SPAWN['drone_3_pos']),

    # genDroneSpawnLaunchDesc('drone_3', 'drone_3_robot_description', DRONE_SPAWN['drone_3_pos']),
    #DRONE 4
    #INTERCOM FOR GATE NODE (DRONE 4)
    # genDroneIntercomLaunchDesc('drone_4', SWARM_SIZE, SWARM_WAYPOINTS, FORMATION_ARG, FORMATION_SPACING,  DRONE_SPAWN['drone_4_pos']),

    # genDroneControllerLaunchDesc('drone_4', SWARM_SIZE, SWARM_WAYPOINTS, FORMATION_ARG, FORMATION_SPACING, DRONE_SPAWN['drone_4_pos']),

    #DRONE 5
    # genDroneControllerLaunchDesc('drone_5', SWARM_SIZE, SWARM_WAYPOINTS, FORMATION_ARG, FORMATION_SPACING, DRONE_SPAWN['drone_5_pos']),
    # #DRONE MULTI-SPAWNER #NOTE: unsure how to add list within this list
    # genDroneMultiSpawnLaunchDesc(SWARM_SIZE, DRONE_SPAWN),
]
#ADD DRONE MULTI-SPAWN
DRONE_DESCRIPTION.extend(genDroneMultiSpawnLaunchDesc(int(SWARM_SIZE), DRONE_SPAWN))

def generate_launch_description():
    
    #START GAZEBO
    start_gazebo = LaunchConfiguration('start_gazebo')
    start_gazebo_launch_arg = DeclareLaunchArgument(
        'start_gazebo',
        default_value='True',
        description='Whether to start Gazebo'
    )

    #START RVIZ
    start_rviz = LaunchConfiguration('start_rviz')
    start_rviz_launch_arg = DeclareLaunchArgument(
        'start_rviz',
        default_value='True',
        description='Whether to start Rviz'
    )

    
    return LaunchDescription([
        # start_rviz_launch_arg,
        
        # DRONE DESCRIPTION (PUBLISH URDF)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory(DESCRIPTION_PKG), "launch", "publish_urdf.launch.py"
                )
            ),
            
            launch_arguments={
                'drone_ns': 'drone_1',
                'drone_description_topic_name': 'drone_1_robot_description',
            }.items(),

        ),
        
        # # DRONE DESCRIPTION (PUBLISH URDF 2)
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         os.path.join(
        #             get_package_share_directory(DESCRIPTION_PKG), "launch", "publish_urdf.launch.py"
        #         )
        #     ),
            
        #     launch_arguments={
        #         'drone_ns': 'drone_2',
        #         'drone_description_topic_name': 'drone_2_robot_description',
        #     }.items(),

        # ),
        
        # # RVIZ
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         os.path.join(
        #             get_package_share_directory(RVIZ_PKG), "launch", 'rviz.launch.py'
        #         )
        #     ),
        #     condition=IfCondition(start_rviz),
        # ),

        # GAZEBO WORLD
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory(GAZEBO_PKG), "launch", 'gazebo.launch.py'
                )
            ),

        ),

        #DRONE SPAWNERS AND CONTROLLERS
        *DRONE_DESCRIPTION
    ])