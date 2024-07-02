import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
#multithreading imports
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

#messages and services imports
from drone_common.srv import BehaviorControl
from drone_common.srv import IMU
from drone_common.srv import GPStoENU
from drone_common.srv import SimClock
from drone_common.srv import VelCalc
from drone_common.srv import DroneIntercomIn
from drone_common.msg import CmdFwd

from scipy.optimize import linear_sum_assignment
import numpy as np
from csv import writer
import os
import re

from .constants import (
    POS_CONTROL,
    DRONE_CONSTANT,
    MPC_CONTROL,
    BEHAVIORAL_CTRL,
    CTRL_DT,
    FORMATION_CONST,
    SWARM_FAILURE, 
)

class CMD_FWD_Publisher(Node):

    def __init__(self):
        super().__init__('cmd_fwd_publisher')

        #MULTITHREADING DECLARATION
        # internal_com_in_service_cb_group = ReentrantCallbackGroup() # MutuallyExclusiveCallbackGroup() # ReentrantCallbackGroup()
        # com_in_subscriber_cb_group = ReentrantCallbackGroup() # MutuallyExclusiveCallbackGroup() # ReentrantCallbackGroup()

        #INITIALIZE COMMAND FORWARD PUBLISHER
        #Service to provide position controller outputs
        # self.cmd_fwd_publisher = self.create_publisher(CmdFwd, '/formation_msg', 10) 
        #NOTE: Create publisher for each respective drone
        self.cmd_fwd_publisher = self.create_publisher(CmdFwd, 'formation_msg', 10) 
        
        timer_period = CTRL_DT['Ts']*2 # seconds #was 1 sec
        self.timer = self.create_timer(timer_period, self.cmd_fwd_publisher_callback)
        
        
        #//////////////////////////////////
        #INITIALIZE REQUIRED CLIENTS 

        self.drone_com_in_sub_node = rclpy.create_node('drone_com_in_sub_node')#create sub-node for imu client
        #Initialize Internal Communication Client
        self.drone_com_in_client = self.drone_com_in_sub_node.create_client(
            DroneIntercomIn, 
            'drone_com_in_service', 
            # callback_group=com_in_subscriber_cb_group
            )
        while not self.drone_com_in_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('drone_com_in service not available, waiting again...')
        self.drone_com_in_req = DroneIntercomIn.Request()


        self.sim_clock_sub_node = rclpy.create_node('sim_clock_sub_node') #create sub-node for sim clock client
        #Initialize sim_clock Client
        self.sim_clock_client = self.sim_clock_sub_node.create_client(
            SimClock, 
            'sim_clock_service'
            )
        while not self.sim_clock_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('SIM CLOCK service not available, waiting again...')
        self.sim_clock_req = SimClock.Request()



        #////////////////////////////////////////////////////////////////////////////
        
        #PARAMETER DECLARATION
        #Get the formation
        self.declare_parameter('formation_arg')
        formation_param = self.get_parameter('formation_arg')
        self.formation_array = formation_param.value
        self.get_logger().info(f'swarm formation_arg parameter: {self.formation_array}')

        #Get the formation spacing
        self.declare_parameter('formation_spacing')
        formation_spacing_param = self.get_parameter('formation_spacing')
        self.form_spacing_array = formation_spacing_param.value
        self.get_logger().info(f'swarm form_spacing parameter: {self.form_spacing_array}')

        #Get the swarm waypoints (position and altitude)
        self.declare_parameter('swarm_waypoints')
        swarm_waypoints_param = self.get_parameter('swarm_waypoints')
        # self.swarm_waypoints = swarm_waypoints_param.value  
        self.swarm_waypoints = [float(value) for value in swarm_waypoints_param.value] # Convert the parameter to a list of floats
        self.get_logger().info(f'drone "swarm_waypoints" parameter: {self.swarm_waypoints}')

        #Get the swarm size
        self.declare_parameter('swarm_size')
        self.swarm_size = self.get_parameter('swarm_size').get_parameter_value().integer_value
        self.get_logger().info(f'drone swarm_size parameter: {self.swarm_size}')
        
        #Get drone namespace to extract its own drone_id
        self.declare_parameter('drone_ns')
        self.drone_namespace = self.get_parameter('drone_ns').get_parameter_value().string_value 
        self.drone_id = re.findall('\d',self.drone_namespace) #look in the drone_id passed (also used for the namespace) and extract the drone id number
        self.drone_id = int(self.drone_id[0]) #convert the string value into an integer

        #////////////////////////////////////////////////////////////////////
        # PARAMETER CALLBACK ON UPDATE
        # Callback to update parameter inputs upon change
        # self.add_on_set_parameters_callback(self.param_callback)

        #////////////////////////////////////////////////////////////////////
        #DRONE SWARM DATA DICTIONARY INTIALIZATION
        self.drone_swarm_dictionary = {}
        for i in range(self.swarm_size): #create a dictionary to hold the drone data of the number of drones specified
            count = i+1
            
            iter_drone_swarm_dictionary = {
                'timestamp_'+ f'{count}': None,
                'velocity_'+ f'{count}': None,
                'position_'+ f'{count}': None, 
                'drone_'+ f'{count}' + '_formation_id': None, 
                }
            self.drone_swarm_dictionary.update(iter_drone_swarm_dictionary) #concatenate dictionary with drone data placeholder
        #////////////////////////////////////////////////////////////////////
        #PASS FORMATION VARIABLES
        #Pass names of formations for match-case
        self.formation_single_file = FORMATION_CONST['formation_names'][0]
        self.formation_single_row = FORMATION_CONST['formation_names'][1]
        self.formation_arrow = FORMATION_CONST['formation_names'][2]
        self.formation_circle = FORMATION_CONST['formation_names'][3]

        #////////////////////////////////////////////////////////////////////
        #INITIALIZE WAYPOINT COUNTER
        #Counter to determine on which goal position will the formation travel to next
        self.waypoint_count = 0
        self.waypoint_form_assign_exec_flag = True #Flag to prevent continuous reassignment of drone formation positions until the next waypoint is reached, then re-assignment occurs again

        self.past_formation = self.form_spacing_array[0] #used to compare if there is a formation change between waypoints.

        #PASS WAYPOINT BOUNDARY RADIUS CONSTANT
        self.BOUND_WAYPT = BEHAVIORAL_CTRL['bound_waypt']
        #PASS ALT BOUNDARY
        self.MIN_FAILURE_BOUND_ALT = SWARM_FAILURE['min_fail_alt_bound']
        self.MAX_FAILURE_BOUND_ALT = SWARM_FAILURE['fail_alt_bound']
        
        self.BOUND_ALT = BEHAVIORAL_CTRL['bound_alt']
        self.BOUND_ALT_POS_CTRL = BEHAVIORAL_CTRL['bound_alt_pos_ctrl']
        self.MIN_BOUND_ALT = BEHAVIORAL_CTRL['min_bound_alt']

        self.ignore_drone_id_array = np.array([0])
        self.form_node_size = self.swarm_size
        self.init_alt_bool = False

    # # PARAMETER UPDATE CALLBACK
    # def param_callback(self, params: list(Parameter)):
    #     for param in params:
    #             if (param.name == 'formation_arg'):
    #                 if (param.value[0] == np.any(FORMATION_CONST['formation_names'])):
    #                     self.formation = param.value[0]
    #                     self.get_logger().info(f'changed formation parameter to: {self.formation}')
    #                 else:
    #                     self.get_logger().info(f'could not change formation parameter, should be from the following selection: {FORMATION_CONST["formation_names"]}')
    #                     return SetParametersResult(successful=False)
                    
    #                 if (param.value[1] >= int(1)):
    #                     self.form_spacing = int(param.value[1])
    #                     self.get_logger().info(f'changed formation spacing parameter to: {self.form_spacing}')
    #                 else:
    #                     self.get_logger().info(f'could not change formation spacing parameter, should be an integer value equal or greater than 1')
    #                     return SetParametersResult(successful=False)
    #     return SetParametersResult(successful=True)



 
        
    def cmd_fwd_publisher_callback(self):
        
        # self.get_logger().info('cmd_fwd_publisher_callback called...')
        
        #CALL SERVICES FOR NECESSARY DATA

        #import Drone Swarm data as client/
        drone_swarm_data = self.drone_com_in_service()

        #import timestamp
        clock_data = self.call_sim_clock_service()
        #self.get_logger().info(f'GPS service data: {clock_data}')

        
        #Pass drone swarm data
        swarm_size = drone_swarm_data.swarm_size #int
        # self.get_logger().info(f'Swarm Size (drone swarm data): {swarm_size}')
        for i in range(swarm_size):
            drone_id = int(i+1)
            index = int(i)
            arr_index = int((i)*3) #index every 3rd element to pull cartesian data of each drone (ie x1,y1,z1, x2,y2,y3)
            self.drone_swarm_dictionary['timestamp_'+f'{drone_id}'] = drone_swarm_data.timestamp[index]
            self.drone_swarm_dictionary['velocity_'+f'{drone_id}'] = drone_swarm_data.velocity[arr_index:(arr_index+3)]
            self.drone_swarm_dictionary['position_'+f'{drone_id}'] = drone_swarm_data.position[arr_index:(arr_index+3)]

        # self.get_logger().info(f'drone_swarm_dictionary: {self.drone_swarm_dictionary}')
        #////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        #GET GOAL POSITIONS AND NEXT FORMATION
        # Get average position of all drones in the world frame.
        #avg_swarm_pos = self.swarm_center_calc(self.swarm_size, self.drone_swarm_dictionary)
        avg_swarm_pos = self.swarm_center_calc(self.form_node_size, self.ignore_drone_id_array, self.swarm_size, self.drone_swarm_dictionary) #updated for drone failure
        #Set formation, spacing and goal position from the array data
        self.formation = self.formation_array[self.waypoint_count]
        self.form_spacing = self.form_spacing_array[self.waypoint_count]

        index = self.waypoint_count*3
        self.swarm_goal_pos = np.array([[self.swarm_waypoints[index]], [self.swarm_waypoints[index+1]], [self.swarm_waypoints[index+2]]])
        # self.swarm_goal_pos = np.array([[self.swarm_waypoints[self.waypoint_count]], [self.swarm_waypoints[self.waypoint_count+1]], [self.swarm_waypoints[self.waypoint_count+2]]])
        #Get the distance from the current formation position to the goal/waypoint position
        form_to_goal_dist_xy =  self.dist_calc_2d(self.swarm_goal_pos, avg_swarm_pos)
        #If the formation is within the goal/waypoint boundary, then set the next point in the array as the goal   
        if (form_to_goal_dist_xy < self.BOUND_WAYPT):
            self.waypoint_count += 1
            self.waypoint_form_assign_exec_flag = True
            swarm_goal_size = len(self.swarm_waypoints)/3
            if (self.waypoint_count <= swarm_goal_size):
                index = self.waypoint_count*3
                self.swarm_goal_pos = np.array([[self.swarm_waypoints[index]], [self.swarm_waypoints[index + 1]], [self.swarm_waypoints[index + 2]]])
            else:
                self.get_logger().info(f'No new waypoint locations, final goal position achieved!!!')
            
            form_name_array_size = len(self.formation_array)
            if (self.waypoint_count <= form_name_array_size):
                self.formation = self.formation_array[self.waypoint_count]
                self.past_formation = self.formation_array[self.waypoint_count-1]
            else:
                self.get_logger().info(f'No further formation commands...')

            form_space_array_size = len(self.form_spacing_array)
            if (self.waypoint_count <= form_space_array_size):
                self.form_spacing = self.form_spacing_array[self.waypoint_count]
            else:
                self.get_logger().info(f'No further formation spacing commands...')
        #////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        #/////////////////////////////////////////////////////////////
        # Get average position of all drones in the world frame.
        # This will be used to determine where to place the formation center
        # avg_swarm_pos = self.swarm_center_calc(self.swarm_size, self.drone_swarm_dictionary) #NOTE: Already calculated
        # self.get_logger().info(f"self.form_node_size 1:  {self.form_node_size} ")
        if (np.absolute(self.swarm_goal_pos[2] - avg_swarm_pos[2]) < self.MIN_BOUND_ALT and self.init_alt_bool == False):
            self.init_alt_bool = True #set bool to true so that drones are checked for failure
            self.waypoint_form_assign_exec_flag = True #set to true so that formation can be accurately set when achieving altitude
        #CHECK FOR ANY DRONES THAT FELL FROM FORMATION
        if (np.absolute(self.swarm_goal_pos[2] - avg_swarm_pos[2]) >= self.MIN_FAILURE_BOUND_ALT and self.init_alt_bool == True):
            swarm_alt_array = np.array([])
            for j in range(swarm_size): #create a dictionary to hold the drone data of the number of drones specified
                count = j+1
                if (count != np.any(self.ignore_drone_id_array)):
                    swarm_alt_array = np.hstack((swarm_alt_array, self.drone_swarm_dictionary['position_'+ f'{count}'][2]))
            
            # self.get_logger().info(f"self.drone_swarm_dictionary:  {self.drone_swarm_dictionary} ")
            # self.get_logger().info(f"swarm_alt_array:  {swarm_alt_array} ")

            min_alt_drone_id = np.argmin(swarm_alt_array)
            min_alt_drone_value = swarm_alt_array[min_alt_drone_id]
            # self.get_logger().info(f"min_alt_drone_value:  {min_alt_drone_value} ")
            if (np.absolute(self.swarm_goal_pos[2] - min_alt_drone_value) > self.MAX_FAILURE_BOUND_ALT):
                self.form_node_size = self.form_node_size - 1 #decrement swarm size
                if (np.any(self.ignore_drone_id_array) == np.any(np.array([0]))):
                    self.ignore_drone_id_array = np.array([])
                self.ignore_drone_id_array = np.hstack((self.ignore_drone_id_array, (min_alt_drone_id + 1) ))
                # self.waypoint_form_assign_exec_flag == True
            # self.get_logger().info(f"self.form_node_size 2:  {self.form_node_size} ")
            # self.get_logger().info(f"self.ignore_drone_id_array:  {self.ignore_drone_id_array} ")
        #/////////////////////////////////////////////////////////////
        # self.get_logger().info(f'waypoint_form_assign_exec_flag: {self.waypoint_form_assign_exec_flag}')
        if (self.swarm_size > 1 and self.waypoint_form_assign_exec_flag == True and self.past_formation != self.formation and (((self.past_formation != self.formation_single_file) and (self.formation != self.formation_single_row)) or ((self.past_formation != self.formation_single_row) and (self.formation != self.formation_single_file )))):
            self.waypoint_form_assign_exec_flag = False #Set flag to false to be reset after reaching the next waypoint
            
            #GET FORMATION POSITIONS IN WORLD FRAME ORIENTED TOWARDS GOAL POSITION
            #if (self.drone_swarm_dictionary['timestamp_'+ f'{1}'] != None): #Check to ensure that
            if any(value is not None for value in self.drone_swarm_dictionary.values()):
                if (self.formation == FORMATION_CONST['formation_names'][0]): #Single File Formation
                    # self.get_logger().info(f'Creating single-file formation...')
                    swarm_formation = self.create_single_file_form(self.form_node_size, self.form_spacing, self.swarm_goal_pos, avg_swarm_pos)
                    # self.get_logger().info(f'Swarm Formation 00: {swarm_formation}')
                elif (self.formation == FORMATION_CONST['formation_names'][1]):
                    # self.get_logger().info(f'Creating row-file formation...')
                    swarm_formation = self.create_single_row_form(self.form_node_size, self.form_spacing, self.swarm_goal_pos, avg_swarm_pos)
                elif (self.formation == FORMATION_CONST['formation_names'][3]):
                    # self.get_logger().info(f'Creating circular formation...')
                    swarm_formation = self.create_circle_form(self.form_node_size, self.form_spacing, self.swarm_goal_pos, avg_swarm_pos)
                else:
                    self.get_logger().info(f'Could not match formation parameter, should be from the following selection: {FORMATION_CONST["formation_names"]}')
                    return
            
                
                #ASSIGN DRONES TO RESPECTIVE FORMATION POSITIONS
                # self.get_logger().info(f'Swarm Formation: {swarm_formation.items()}') #check dictionary
                self.drone_swarm_dictionary = self.drone_assignment_calc(self.form_node_size, self.ignore_drone_id_array, self.swarm_size, self.drone_swarm_dictionary, swarm_formation)

                

                    # self.get_logger().info(f'passing messages to publisher...')
                # self.get_logger().info(f'passing messages to publisher...')
                #PASS COMMANDS TO OTHER DRONES THROUGH TOPIC
                msg = CmdFwd()
                msg.formation_arg = self.formation_array
                # self.get_logger().info(f'self.formation_array: {self.formation_array}')
                msg.formation_spacing = self.form_spacing_array
                # self.get_logger().info(f'self.form_spacing_array: {self.form_spacing_array}')
                msg.swarm_waypoints = self.swarm_waypoints
                # self.get_logger().info(f'self.swarm_waypoints: {self.swarm_waypoints}')
                msg.form_node_size = self.form_node_size
                
                #self.ignore_drone_id_array.astype(int)
                msg.ignore_drone_id_array = [int(x) for x in self.ignore_drone_id_array] #self.ignore_drone_id_array.tolist()
                
                drone_formation_id_array = np.zeros([self.swarm_size])
                for i in range(self.swarm_size): #cycle through each drone
                    drone_count = i
                    drone_id = i+1
                    drone_formation_id_array[drone_count] = self.drone_swarm_dictionary['drone_'+ f'{drone_id}'+'_formation_id'] #get assignment of drones to the formation

                drone_formation_id_array = [int(x) for x in drone_formation_id_array]# Convert array elements to integers
                msg.drone_formation_id_array = drone_formation_id_array #send array of formation positions
                # self.get_logger().info(f'drone_formation_id_array: {drone_formation_id_array}')
                #publish the message
                self.cmd_fwd_publisher.publish(msg)
            elif(self.swarm_size == 1 and self.waypoint_form_assign_exec_flag == True):
                self.waypoint_form_assign_exec_flag = False 
                self.drone_swarm_dictionary['drone_'+ f'{1}'+'_formation_id'] = 1
                #PASS COMMANDS TO OTHER DRONES THROUGH TOPIC
                msg = CmdFwd()
                msg.formation_arg = self.formation_array
                # self.get_logger().info(f'self.formation_array: {self.formation_array}')
                msg.formation_spacing = self.form_spacing_array
                # self.get_logger().info(f'self.form_spacing_array: {self.form_spacing_array}')
                msg.swarm_waypoints = self.swarm_waypoints
                # self.get_logger().info(f'self.swarm_waypoints: {self.swarm_waypoints}')
                msg.form_node_size = self.form_node_size
                
                #self.ignore_drone_id_array.astype(int)
                msg.ignore_drone_id_array = [int(x) for x in self.ignore_drone_id_array] #self.ignore_drone_id_array.tolist()
                
                drone_formation_id_array = np.zeros([self.swarm_size])
                for i in range(self.swarm_size): #cycle through each drone
                    drone_count = i
                    drone_id = i+1
                    drone_formation_id_array[drone_count] = self.drone_swarm_dictionary['drone_'+ f'{drone_id}'+'_formation_id'] #get assignment of drones to the formation

                drone_formation_id_array = [int(x) for x in drone_formation_id_array]# Convert array elements to integers
                msg.drone_formation_id_array = drone_formation_id_array #send array of formation positions
                # self.get_logger().info(f'drone_formation_id_array: {drone_formation_id_array}')
                #publish the message
                self.cmd_fwd_publisher.publish(msg)

            else: 
                #PASS COMMANDS TO OTHER DRONES THROUGH TOPIC
                msg = CmdFwd()
                msg.formation_arg = self.formation_array
                msg.formation_spacing = self.form_spacing_array
                msg.swarm_waypoints = self.swarm_waypoints
                msg.form_node_size = self.form_node_size
                # self.ignore_drone_id_array.astype(int)
                msg.ignore_drone_id_array = [int(x) for x in self.ignore_drone_id_array] #self.ignore_drone_id_array.tolist()
                
                #publish the message
                self.cmd_fwd_publisher.publish(msg)
                return 

        else:
            # self.get_logger().info(f'passing messages to publisher...')
            if (self.swarm_size == 1):
                self.drone_swarm_dictionary['drone_'+ f'{1}'+'_formation_id'] = 1
            #PASS COMMANDS TO OTHER DRONES THROUGH TOPIC
            msg = CmdFwd()
            msg.formation_arg = self.formation_array
            # self.get_logger().info(f'self.formation_array: {self.formation_array}')
            msg.formation_spacing = self.form_spacing_array
            # self.get_logger().info(f'self.form_spacing_array: {self.form_spacing_array}')
            msg.swarm_waypoints = self.swarm_waypoints
            # self.get_logger().info(f'self.swarm_waypoints: {self.swarm_waypoints}')

            msg.form_node_size = self.form_node_size
            # self.ignore_drone_id_array.astype(int)
            msg.ignore_drone_id_array = [int(x) for x in self.ignore_drone_id_array] #self.ignore_drone_id_array.tolist()
            
            drone_formation_id_array = np.zeros([self.swarm_size])
            for i in range(self.swarm_size): #cycle through each drone
                drone_count = i
                drone_id = i+1
                drone_formation_id_array[drone_count] = self.drone_swarm_dictionary['drone_'+ f'{drone_id}'+'_formation_id'] #get assignment of drones to the formation

            drone_formation_id_array = [int(x) for x in drone_formation_id_array]# Convert array elements to integers
            msg.drone_formation_id_array = drone_formation_id_array #send array of formation positions
            # self.get_logger().info(f'drone_formation_id_array: {drone_formation_id_array}')
            #publish the message
            self.cmd_fwd_publisher.publish(msg)

            self.waypoint_form_assign_exec_flag = False #set waypoint flag to false in case second requirement in the if statement is unfulfilled
            

    # /////////////////////////////////////////////////////////////

    def drone_assignment_calc(self, form_node_size, ignore_drone_id_array, swarm_size, drone_swarm_dictionary, swarm_formation):
        if swarm_formation is None:
            # Handle the case when swarm_formation is None
            # You can raise an exception, return a default value, or perform other actions based on your requirement.
            raise ValueError("Swarm formation is not available. Has not been filled yet.")
        
        # self.get_logger().info(f'drone_swarm_dict: {drone_swarm_dictionary}')
        # self.get_logger().info(f'form_pos_dict: {swarm_formation}')
        #find smallest distance to each formation location and assign the formation positions to the drones closest

        #find distances of all robots to each formation position

        if (form_node_size == swarm_size):
            drone2form_dist = np.zeros([swarm_size, swarm_size]) #initialize matrix (drone positions, formation positions)

            drone2form_dist_center = np.zeros([swarm_size, 1]) #initalize matrix for calculating distance to the formation center
            for i in range(swarm_size): #cycle through each drone
                drone_count = i
                drone_id = i+1
                drone_pos_array = drone_swarm_dictionary['position_'+ f'{drone_id}'] #get drone position array
                
                # drone2form_dist_center[drone_count] = self.dist_calc_2d(form_center_pos, drone_pos_array) # Determine distance of each drone to the center of the formation
                
                for j in range(swarm_size): #Cycle through each formation position
                    form_pos_count = j
                    form_id_iter = j+1
                    form_pos_array = swarm_formation['form_uav'+ f'{form_id_iter}'] #Get formation position array
                    
                    drone2form_dist[drone_count, form_pos_count] = self.dist_calc_2d(form_pos_array, drone_pos_array) #calc distance from drone to formation
        else:
            drone2form_dist = np.zeros([swarm_size, form_node_size])

            for i in range(swarm_size): #cycle through each drone
                drone_count = i
                drone_id = i+1
                drone_pos_array = drone_swarm_dictionary['position_'+ f'{drone_id}'] #get drone position array
                
                # drone2form_dist_center[drone_count] = self.dist_calc_2d(form_center_pos, drone_pos_array) # Determine distance of each drone to the center of the formation
                
                for j in range(form_node_size): #Cycle through each formation position
                    form_pos_count = j
                    form_id_iter = j+1
                    form_pos_array = swarm_formation['form_uav'+ f'{form_id_iter}'] #Get formation position array
                    
                    drone2form_dist[drone_count, form_pos_count] = self.dist_calc_2d(form_pos_array, drone_pos_array) #calc distance from drone to formation

            modified_matrix = drone2form_dist.copy()
            for k in ignore_drone_id_array:
                id = int(k)
                modified_matrix[(id - 1), :] = np.max(drone2form_dist) + 1 
                drone_swarm_dictionary['drone_'+ f'{id}' + '_formation_id'] = 0
            
        # print(drone2form_dist)
        # print(drone2form_dist_center)
        # self.get_logger().info(f'drone_formation_dist: {drone2form_dist}')
        # Use the Hungarian algorithm to find the optimal assignment
        row_ind, col_ind = linear_sum_assignment(drone2form_dist)
        # Assign drones to positions based on the optimal assignment
        for drone, position in zip(row_ind, col_ind):
            
            #Set the drone with the smallest distance to the assigned position
            drone_swarm_dictionary['drone_'+ f'{drone+1}' + '_formation_id'] = position + 1

        # for i in range(swarm_size): #cycle through each drone
        #     drone2form_dist_min_index = np.array(np.unravel_index(np.argmin(drone2form_dist, axis=None), drone2form_dist.shape)) #Convert matrix to linear array, then get the first encountered lowest value, then return it's position. Convert indices back into the matrix form of the shape given.
        #     #print(drone2form_dist_min_index) #row is the corresponding drone, column is the corresponding position
        #     #Set the drone with the smallest distance to the assigned position
        #     drone_swarm_dictionary['drone_'+ f'{drone2form_dist_min_index[0]+1}' + '_formation_id'] = drone2form_dist_min_index[1] + 1
        #     drone2form_dist[drone2form_dist_min_index[0]] = np.inf #Set row of the selected drone to inf to not be used in determining min
        #     drone2form_dist[:,drone2form_dist_min_index[1]] = np.inf  #Set column of the selected formation position to inf to not be used in determining min
        #     # print(drone_swarm_dictionary)
            # # print(drone2form_dist)

        self.get_logger().info(f'drone_swarm_dict POST-FORM-ASSIGN: {drone_swarm_dictionary}')
        
        return drone_swarm_dictionary #return updated formation id
    
    def swarm_center_calc(self, form_node_size, ignore_drone_id_array,  swarm_size, drone_swarm_dictionary): #Find center of drone swarm from current positions
        avg_swarm_pos = np.zeros_like(drone_swarm_dictionary['position_'+ f'{1}']) #initialize avg_swarm_pos size to the same as drone_swarm_dictionary[position_1] (NOTE: the selected position is arbitrary, only variable size is needed)
        for j in range(swarm_size): #create a dictionary to hold the drone data of the number of drones specified
            count = j+1
            
            if (count != np.any(ignore_drone_id_array)):
                avg_swarm_pos = np.add(avg_swarm_pos, drone_swarm_dictionary['position_'+ f'{count}'])
            
        # avg_swarm_pos = avg_swarm_pos/swarm_size
        avg_swarm_pos = avg_swarm_pos/form_node_size #updated to consider varying formation nodes in swarm

        avg_swarm_pos = np.array([[avg_swarm_pos[0]], [avg_swarm_pos[1]], [avg_swarm_pos[2]]]) #change into a column vector for further calculations
        return avg_swarm_pos
        #print(avg_swarm_pos)

    def formation_center_calc(self, swarm_size, swarm_formation):  
        form_center_pos = np.zeros([3,1])

        for j in range(swarm_size): 
            count = j+1
            
            form_center_pos = np.add(form_center_pos, swarm_formation['form_uav'+ f'{count}'])

        form_center_pos = form_center_pos/swarm_size
        return form_center_pos
        #print(form_center_pos)

    def translate_formation(self, swarm_size, swarm_formation, vect_translate):
        for i in range(swarm_size): #create a dictionary to hold the drone data of the number of drones specified
            id_iter = int(i+1)
        

            swarm_formation_pos_vect = swarm_formation['form_uav'+ f'{id_iter}']
            #print(swarm_formation_pos_vect)
            swarm_formation['form_uav'+ f'{id_iter}'] = swarm_formation_pos_vect + vect_translate
                
        #print(swarm_formation)
        return swarm_formation
    
    def rotate_formation_z(self, swarm_size, swarm_formation, form2goal_angle, avg_swarm_pos):
        #Find the new points of the rotated formation about swarm origin, then convert to inertial frame coordinates
        for i in range(swarm_size): #create a dictionary to hold the drone data of the number of drones specified
            form_id_iter = int(i+1)
            
            swarm_formation_pos_vect = swarm_formation['form_uav'+ f'{form_id_iter}'] - avg_swarm_pos #convert to swarm origin
            # print(swarm_formation_pos_vect)
            swarm_formation['form_uav'+ f'{form_id_iter}'] = self.Rot_z(swarm_formation_pos_vect,form2goal_angle) #rotate about swarm origin
            swarm_formation['form_uav'+ f'{form_id_iter}'] = swarm_formation['form_uav'+ f'{form_id_iter}'] + avg_swarm_pos #convert back to inertial frame
        #print(swarm_formation)
        return swarm_formation

    def create_circle_form(self, swarm_size, formation_spacing, swarm_goal_pos, avg_swarm_pos):

        # Create Circular formation
        ang_form_spacing = 2*(np.pi)/(swarm_size) #Determine angular spacing for drones to be equally distributed
        swarm_formation = {}
        for i in range(swarm_size): #create a dictionary to hold the drone data of the number of drones specified
            form_id_iter = i+1
            count = i

            iter_swarm_formation = {
                'form_uav'+ f'{form_id_iter}': np.array([[formation_spacing*np.cos(count*ang_form_spacing)], [formation_spacing*np.sin(count*ang_form_spacing)], [0.0]]), 
                }
            swarm_formation.update(iter_swarm_formation) #concatenate dictionary with drone data placeholder
                
        #print(swarm_formation)

        #----------------------------------------------------------------
        #Translate formation center to the drone swarm center
        swarm_formation = self.translate_formation(swarm_size, swarm_formation, avg_swarm_pos)
        #----------------------------------------------------------------
        #get normalized vector that depicts the formation by creating a vector from the 1st and last drone positions (id=1 to id=swarm_size) at the current drone swarm center
        form_lead_vector = swarm_formation['form_uav'+ f'{1}'] - avg_swarm_pos#swarm_formation['form_uav'+ f'{swarm_size}']#translate only the 1st position/drone_id=1 of the drone formation
        #print(form_lead_vector)
        form_lead_vector = form_lead_vector/(np.sqrt((form_lead_vector[0]**2)+(form_lead_vector[1]**2)+(form_lead_vector[2]**2)))
        form_lead_vector_row =  np.array([form_lead_vector[0,0], form_lead_vector[1,0], form_lead_vector[2,0]])
        #print(form_lead_vector)
        #print(form_lead_vector_row)
        #----------------------------------------------------------------
        #get normalized vector to the goal point from the drone swarm center
        swarm_goal_loc_row =  np.array([swarm_goal_pos[0,0] - avg_swarm_pos[0,0], swarm_goal_pos[1,0] - avg_swarm_pos[1,0], 0.0 - avg_swarm_pos[2,0]])
        #print(swarm_goal_loc_row)
        swarm_goal_loc_row = swarm_goal_loc_row/(np.sqrt((swarm_goal_loc_row[0]**2)+(swarm_goal_loc_row[1]**2)+(swarm_goal_loc_row[2]**2)))
        #print(swarm_goal_loc_row)

        #get the angle between the line formation and the goal point
        form2goal_angle = self.vector_angle_2d_swarm_form(form_lead_vector_row, swarm_goal_loc_row)
        #print(form2goal_angle*(180/np.pi))
        #----------------------------------------------------------------
        #Find the new points of the rotated formation about swarm origin, then convert to inertial frame coordinates
        swarm_formation = self.rotate_formation_z(swarm_size, swarm_formation, form2goal_angle, avg_swarm_pos)
        
        return swarm_formation

    def create_single_file_form(self, swarm_size, formation_spacing, swarm_goal_pos, avg_swarm_pos):

        # Create Single file vertical line
        swarm_formation = {}
        for i in range(swarm_size): #create a dictionary to hold the drone data of the number of drones specified
            form_id_iter = i+1
            count = i

            iter_swarm_formation = {
                'form_uav'+ f'{form_id_iter}': np.array([[0.0], [-1*count*formation_spacing], [0.0]]), 
                }
            swarm_formation.update(iter_swarm_formation) #concatenate dictionary with drone data placeholder
                
        #print(swarm_formation)
        # self.get_logger().info(f'Swarm Formation 1: {swarm_formation.items()}')
        #----------------------------------------------------------------

        #----------------------------------------------------------------
        #find distance to the formation center from inertial frame origin
        form_center_pos = self.formation_center_calc(swarm_size, swarm_formation)
        # self.get_logger().info(f'Swarm Formation 2: {swarm_formation.items()}')
        #----------------------------------------------------------------
        #Translate formation center location to the inertial frame origin
        swarm_formation = self.translate_formation(swarm_size, swarm_formation, np.absolute(form_center_pos))
        # self.get_logger().info(f'Swarm Formation 3: {swarm_formation.items()}')
        #----------------------------------------------------------------
        #Translate formation center to the drone swarm center
        swarm_formation = self.translate_formation(swarm_size, swarm_formation, avg_swarm_pos)
        # self.get_logger().info(f'Swarm Formation 4: {swarm_formation.items()}')
        #----------------------------------------------------------------
        #get normalized vector that depicts the formation by creating a vector from the 1st and last drone positions (id=1 to id=swarm_size) at the current drone swarm center
        form_lead_vector = swarm_formation['form_uav'+ f'{1}'] - avg_swarm_pos#swarm_formation['form_uav'+ f'{swarm_size}']#translate only the 1st position/drone_id=1 of the drone formation
        #print(form_lead_vector)
        form_lead_vector = form_lead_vector/(np.sqrt((form_lead_vector[0]**2)+(form_lead_vector[1]**2)+(form_lead_vector[2]**2)))
        form_lead_vector_row =  np.array([form_lead_vector[0,0], form_lead_vector[1,0], form_lead_vector[2,0]])
        #print(form_lead_vector)
        #print(form_lead_vector_row)
        #----------------------------------------------------------------
        #get normalized vector to the goal point from the drone swarm center
        swarm_goal_loc_row =  np.array([swarm_goal_pos[0,0] - avg_swarm_pos[0,0], swarm_goal_pos[1,0] - avg_swarm_pos[1,0], 0.0 - avg_swarm_pos[2,0]])
        #print(swarm_goal_loc_row)
        swarm_goal_loc_row = swarm_goal_loc_row/(np.sqrt((swarm_goal_loc_row[0]**2)+(swarm_goal_loc_row[1]**2)+(swarm_goal_loc_row[2]**2)))
        #print(swarm_goal_loc_row)

        #get the angle between the line formation and the goal point
        form2goal_angle = self.vector_angle_2d_swarm_form(form_lead_vector_row, swarm_goal_loc_row)
        #print(form2goal_angle*(180/np.pi))
        #----------------------------------------------------------------
        #Find the new points of the rotated formation about swarm origin, then convert to inertial frame coordinates
        swarm_formation = self.rotate_formation_z(swarm_size, swarm_formation, form2goal_angle, avg_swarm_pos)
        # self.get_logger().info(f'Swarm Formation 5: {swarm_formation.items()}')
        
        return swarm_formation
    
    def create_single_row_form(self, swarm_size, formation_spacing, swarm_goal_pos, avg_swarm_pos):
                
        # Create Single file vertical line
        swarm_formation = {}
        for i in range(swarm_size): #create a dictionary to hold the drone data of the number of drones specified
            form_id_iter = i+1
            count = i
            iter_swarm_formation = {
                'form_uav'+ f'{form_id_iter}': np.array([[0.0], [-1*count*formation_spacing], [0.0]]), 
                }
            swarm_formation.update(iter_swarm_formation) #concatenate dictionary with drone data placeholder

        #print(swarm_formation)
        #----------------------------------------------------------------
        #----------------------------------------------------------------
        #find distance to the formation center from inertial frame origin
        form_center_pos = self.formation_center_calc(swarm_size, swarm_formation)
        #----------------------------------------------------------------
        #Translate formation center location to the inertial frame origin
        swarm_formation = self.translate_formation(swarm_size, swarm_formation, np.absolute(form_center_pos))
        #----------------------------------------------------------------
        #Translate formation center to the drone swarm center
        swarm_formation = self.translate_formation(swarm_size, swarm_formation, avg_swarm_pos)
        #----------------------------------------------------------------
        #get normalized vector that depicts the formation by creating a vector from the 1st and last drone positions (id=1 to id=swarm_size) at the current drone swarm center
        form_lead_vector = swarm_formation['form_uav'+ f'{1}'] - avg_swarm_pos#swarm_formation['form_uav'+ f'{swarm_size}']#translate only the 1st position/drone_id=1 of the dr
        #print(form_lead_vector)
        form_lead_vector = form_lead_vector/(np.sqrt((form_lead_vector[0]**2)+(form_lead_vector[1]**2)+(form_lead_vector[2]**2)))
        form_lead_vector_row =  np.array([form_lead_vector[0,0], form_lead_vector[1,0], form_lead_vector[2,0]])
        #print(form_lead_vector)
        #print(form_lead_vector_row)
        #----------------------------------------------------------------
        #get normalized vector to the goal point from the drone swarm center
        swarm_goal_loc_row =  np.array([swarm_goal_pos[0,0] - avg_swarm_pos[0,0], swarm_goal_pos[1,0] - avg_swarm_pos[1,0], 0.0 - avg_swarm_pos[2,0]])
        #print(swarm_goal_loc_row)
        swarm_goal_loc_row = swarm_goal_loc_row/(np.sqrt((swarm_goal_loc_row[0]**2)+(swarm_goal_loc_row[1]**2)+(swarm_goal_loc_row[2]**2)))
        #print(swarm_goal_loc_row)
        #get the angle between the line formation and the goal point
        form2goal_angle = self.vector_angle_2d_swarm_form(form_lead_vector_row, swarm_goal_loc_row)
        #print(form2goal_angle*(180/np.pi))
        #----------------------------------------------------------------
        #Find the new points of the rotated formation about swarm origin, then convert to inertial frame coordinates
        swarm_formation = self.rotate_formation_z(swarm_size, swarm_formation, form2goal_angle, avg_swarm_pos)

        return swarm_formation
    # /////////////////////////////////////////////////////////////


    def dist_calc_2d(self, Vgoal, Vref):
        """
        Calculate distance along x-y plane, taking two vectors of at least size = 2
        """
        dist = np.sqrt((Vgoal[0] - Vref[0])**2 + (Vgoal[1] - Vref[1])**2)

        #((self.GOAL_LOC_XY[0] - self.drone_pos_if[0])**2 + (self.GOAL_LOC_XY[1] - self.drone_pos_if[1])**2)**(0.5)
        return dist
    
    def vector_angle_2d_swarm_form(self, Vref_x, Vb):
        #Uses the right hand rule about the z-axis. Considering the x-axis as the reference
        #NOTE: ALL VECTORS ARE ROW VECTORS
        
        #input two column vector arrays in 2-d that are normalized
        Vn = np.array([0.0,0.0,1.0]) #normal vector in z-axis
        angle = np.arctan2(np.dot(np.cross(Vref_x, Vb), Vn), np.dot(Vref_x, Vb)) #result in radians
        # if (angle < 0):
            # angle = 2*np.pi + angle
        #angle = angle*(180/np.pi)
        #print(angle)
        return angle

    def Rot_z(self, Vb_rt, psi_rt):
        # Rotational matrix that rotates about Z axis of frame
        #Phi - roll; Theta - pitch; Psi - Yaw
        R_z = np.array([[np.cos(psi_rt),-np.sin(psi_rt),0],[np.sin(psi_rt),np.cos(psi_rt),0],[0,0,1]])

        Vi_rt = np.matmul(R_z, Vb_rt)

        return Vi_rt

    def Rot_bf_to_if(self, Vb_rt, phi_rt, theta_rt, psi_rt):
        # Rotational matrix that relates body frame velocity to inertial frame velocity
        #Phi - roll; Theta - pitch; Psi - Yaw
        R_x = np.array([[1, 0, 0],[0, np.cos(phi_rt), -np.sin(phi_rt)],[0, np.sin(phi_rt), np.cos(phi_rt)]])
        R_y = np.array([[np.cos(theta_rt),0,np.sin(theta_rt)],[0,1,0],[-np.sin(theta_rt),0,np.cos(theta_rt)]])
        R_z = np.array([[np.cos(psi_rt),-np.sin(psi_rt),0],[np.sin(psi_rt),np.cos(psi_rt),0],[0,0,1]])

        R_matrix = np.matmul(R_z,np.matmul(R_y,R_x))

        Vi_rt = np.matmul(R_matrix, Vb_rt)
    
        return Vi_rt

    def drone_com_in_service(self):
        
        self.drone_com_in_future = self.drone_com_in_client.call_async(self.drone_com_in_req) 
        rclpy.spin_until_future_complete(self.drone_com_in_sub_node, self.drone_com_in_future)
        
        return self.drone_com_in_future.result()
    
    
    def call_sim_clock_service(self):

        self.sim_clock_future = self.sim_clock_client.call_async(self.sim_clock_req)
        rclpy.spin_until_future_complete(self.sim_clock_sub_node, self.sim_clock_future)
        return self.sim_clock_future.result()




def main(args=None):
    rclpy.init(args=args)

    cmd_fwd_publisher = CMD_FWD_Publisher()

    rclpy.spin(cmd_fwd_publisher)

    #/////////////////////////
    #MULTI-THREADING CALLBACKS OPTION
    # executor = MultiThreadedExecutor() #add multi-threading to the node
    # executor.add_node(cmd_fwd_publisher)
    # try:
    #     cmd_fwd_publisher.get_logger().info('Beginning client, shut down with CTRL-C')
    #     executor.spin()
    # except KeyboardInterrupt:
    #     cmd_fwd_publisher.get_logger().info('Keyboard interrupt, shutting down.\n')
    #///////////////////////////

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    cmd_fwd_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()