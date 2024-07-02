import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

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
)

class BehaviorController(Node):

    def __init__(self):
        super().__init__('behavior_controller')


        #INITIALIZE BEHAVIORAL CONTROL SERVICE
        #Service to provide position controller outputs
        self.srv = self.create_service(BehaviorControl, 'behavior_ctrl_service', self.behavior_controller_service_callback)
        
        
        #INITIALIZE REQUIRED SUBSCRIBERS
        #//////////////////////////////////
 
        #Create subscriber for each respective drone
        self.subscription = self.create_subscription(
            CmdFwd,
            'formation_msg', #call '/formation_msg' absolute (with "/" i.e. '/gps') to ensure the namespace isn't attached
            self.cmd_fwd_listener_callback, 
            10
        )
        
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


        self.imu_sub_node = rclpy.create_node('imu_sub_node')#create sub-node for imu client
        #Initialize IMU Client
        self.imu_client = self.imu_sub_node.create_client(IMU, 'IMU_data_service')
        while not self.imu_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('IMU service not available, waiting again...')
        self.imu_req = IMU.Request()
        
        self.gps_sub_node = rclpy.create_node('gps_sub_node') #create sub-node for gps client
        #Initialize GPS Client
        self.gps_client = self.gps_sub_node.create_client(GPStoENU, 'gps_WGS84_to_enu_service')
        while not self.gps_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('GPS service not available, waiting again...')
        self.gps_req = GPStoENU.Request()

        self.sim_clock_sub_node = rclpy.create_node('sim_clock_sub_node') #create sub-node for sim clock client
        #Initialize sim_clock Client
        self.sim_clock_client = self.sim_clock_sub_node.create_client(SimClock, 'sim_clock_service')
        while not self.sim_clock_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('SIM CLOCK service not available, waiting again...')
        self.sim_clock_req = SimClock.Request()

        self.vel_calc_sub_node = rclpy.create_node('vel_sub_node') #create sub-node for vel calc client
        #Initialize Vel Calc Client
        self.vel_calc_client = self.vel_calc_sub_node.create_client(VelCalc, 'vel_calc_service')
        while not self.vel_calc_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('VEL CALC service not available, waiting again...')
        self.vel_calc_req = VelCalc.Request()
        
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

        #Get the swarm waypoints
        self.declare_parameter('swarm_waypoints')
        swarm_waypoints_param = self.get_parameter('swarm_waypoints')
        self.swarm_waypoints = swarm_waypoints_param.value
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

        #DRONE SWARM DATA DICTIONARY INTIALIZATION
        self.drone_swarm_dictionary = {}
        for i in range(self.swarm_size): #create a dictionary to hold the drone data of the number of drones specified
            count = i+1
            
            iter_drone_swarm_dictionary = {
                'timestamp_'+ f'{count}': 0.0,
                'velocity_'+ f'{count}': [0.0,0.0, 0.0],
                'position_'+ f'{count}': [0.0, 0.0, 0.0], 
                }
            self.drone_swarm_dictionary.update(iter_drone_swarm_dictionary) #concatenate dictionary with drone data placeholder

        
        self.swarm_formation = {}
        none_column_array = np.empty((3, 1), dtype=object)
        none_column_array[:] = None
        for i in range(self.swarm_size): #create a dictionary to hold the drone data of the number of drones specified. Initialized to none
            form_id_iter = i+1

            iter_swarm_formation = {
                'form_uav'+ f'{form_id_iter}': none_column_array, 
                }
            self.swarm_formation.update(iter_swarm_formation) #concatenate dictionary with drone data placeholder
        self.goal_form_pos = self.swarm_formation['form_uav'+ f'{1}']
        #///////////////////////////////////////////////////////////////////////////////
        #PASS FORMATION VARIABLES
        #Pass names of formations for match-case
        self.formation_single_file = FORMATION_CONST['formation_names'][0]
        self.formation_single_row = FORMATION_CONST['formation_names'][1]
        self.formation_arrow = FORMATION_CONST['formation_names'][2]
        self.formation_circle = FORMATION_CONST['formation_names'][3]


        #///////////////////////////////////////////////////////////////////////////////
        #Initialize variables
        self.past_swarm_formation_at_waypoint_goal = {}
        self.past_swarm_goal_pos = np.array([[None],[None],[None]])
        self.ascent_descent_bool = False
        self.ascent_descent_goal_pos = np.zeros([3, 1])

        self.new_waypoint_bool = False
        self.form_rotate_bool = False
        self.form_rotate_goal_pos = np.zeros([3, 1])
        self.past_formation = self.formation_array[0]
        self.past_timestamp = 0.0

        self.current_alt = 0.0

        self.drone_lin_vel_bf = np.zeros([3, 1])
        self.drone_lin_vel_if = np.zeros([3, 1])
        self.drone_orient = np.zeros([4, 1])
        self.drone_pos_if = np.zeros([3, 1])
        self.drone_lin_accel_bf = np.zeros([3, 1])
        self.drone_lin_accel_if = np.zeros([3, 1])

        self.f_weights = np.zeros([1, 5])
        self.unit_vector_array = np.array([[0.0, 0.0, 0.0],[0.0, 0.0, 0.0],[0.0, 0.0, 0.0],[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])
        
        self.GRAVITY = DRONE_CONSTANT['g']
        self.MASS = DRONE_CONSTANT['mass']

        self.past_behavior_ctrl_out_vel = np.zeros([3, 1])
        self.past_behavior_ctrl_out_pos = None
        
        #Counter to determine on which goal position will the formation travel to next
        self.waypoint_count = 0

        #//////////////////////////////////

        #time step controller iterates
        self.CTRL_TIME_STEP = CTRL_DT['Ts']*4 #BEHAVIORAL_CTRL['controller_time_step']
        #angle from the x-axis of the body frame to the centerline of the quadrotor counter-clockwise
        self.ANG_X_TO_C = BEHAVIORAL_CTRL['alpha_f']
        #Altitude maintenance
        self.W_ALT = BEHAVIORAL_CTRL['w_alt']
        self.BOUND_ALT = BEHAVIORAL_CTRL['bound_alt']
        self.BOUND_ALT_POS_CTRL = BEHAVIORAL_CTRL['bound_alt_pos_ctrl']
        self.MIN_BOUND_ALT = BEHAVIORAL_CTRL['min_bound_alt']

        #Formation behavior
        self.W_FORM = BEHAVIORAL_CTRL['w_form']
        self.BOUND_FORM = BEHAVIORAL_CTRL['bound_form']
        self.MIN_BOUND_FORM = BEHAVIORAL_CTRL['min_bound_form']
        self.form_assign_node_id = None #Set to none to ensure the behavior formation isn't calculated until a formation location has been assigned. otherwise behavior does nothing
        #Waypoint
        self.BOUND_WAYPT = BEHAVIORAL_CTRL['bound_waypt']
        self.MIN_WAYPOINT_BOUND = BEHAVIORAL_CTRL['min_waypoint_bound']
        #Goal behavior
        self.W_GOAL = BEHAVIORAL_CTRL['w_goal']
        self.BOUND_GOAL = BEHAVIORAL_CTRL['bound_goal']
        self.MIN_BOUND_GOAL = BEHAVIORAL_CTRL['min_bound_goal']
        #ascent/descent behavior
        self.W_CLIMB = BEHAVIORAL_CTRL['w_climb']
        self.BOUND_CLIMB = BEHAVIORAL_CTRL['bound_climb']
        self.MIN_BOUND_CLIMB = BEHAVIORAL_CTRL['min_bound_climb']
        #Collision Avoidance
        self.W_AVOID_UAV = BEHAVIORAL_CTRL['w_avoid_uav']
        self.BOUND_AVOID_UAV = BEHAVIORAL_CTRL['bound_avoid_uav']
        self.MIN_BOUND_AVOID_UAV = BEHAVIORAL_CTRL['min_bound_avoid_uav']
        self.COLLISION_BOUND_AVOID_UAV = BEHAVIORAL_CTRL['collision_bound_avoid_uav']

        # #NOTE:TEMPORARY: Altitude goal
        # self.GOAL_ALT = BEHAVIORAL_CTRL['goal_alt']
        # #NOTE:TEMPORARY: X-Y goal location
        # self.GOAL_LOC_XY = BEHAVIORAL_CTRL['goal_loc_xy']

        #Max linear velocity 
        self.MAX_LIN_VEL_Z = BEHAVIORAL_CTRL['max_lin_vel_z']
        self.MAX_LIN_VEL_XY = BEHAVIORAL_CTRL['max_lin_vel_xy']

        #Max angular velocity 
        self.MAX_YAW_ANG_VEL = BEHAVIORAL_CTRL['max_yaw_ang_vel']
        #Boundary about yaw goal position
        self.BOUND_ANG_YAW = BEHAVIORAL_CTRL['bound_ang_yaw']
        #yaw weight
        self.W_YAW = BEHAVIORAL_CTRL['w_yaw']

        #Max linear acceleration z-axis (if) 
        self.MAX_LIN_ACCEL_Z = BEHAVIORAL_CTRL['max_lin_accel_z']

        #Max linear acceleration xy-axis (if) 
        self.MAX_LIN_ACCEL_XY = BEHAVIORAL_CTRL['max_lin_accel_xy']


        #Initialize iteration counter for csv write to initialize file, then later add variables
        self.iter_count = 0.0
        self.iter_count_yaw = 0.0

        #FAILURE PREVENTION VARIABLES
        self.form_node_size = self.swarm_size
        self.ignore_drone_id_array = np.array([])

    def behavior_controller_service_callback(self, request, response): 
        #self.get_logger().info('behavior_controller_service_callback...')
        #calculate new velocity command trajectory
        behavior_control_data = self.behavior_controller_calc()
        
        #Pass next goal velocity command (array)
        behavior_ctrl_out_vel = behavior_control_data[0] #pass velocity command data
        
        #Pass goal yaw position
        response.goal_psi_y_if = float(behavior_control_data[1]) #pass yaw reference command
        #Pass goal yaw angular velocity
        response.goal_ang_vel_psi_if = float(behavior_control_data[5])
        #Pass final yaw position goal based on commanded velocity orientation
        response.vel_cmd_goal_yaw_if = float(behavior_control_data[6])
        #Pass formation goal position and altitude
        swarm_goal_pos = behavior_control_data[7]
        response.swarm_goal_pos = [float(swarm_goal_pos[0]), float(swarm_goal_pos[1]), float(swarm_goal_pos[2])]
        
        response.past_behavior_ctrl_out_vel_x = float(self.past_behavior_ctrl_out_vel[0])
        response.past_behavior_ctrl_out_vel_y = float(self.past_behavior_ctrl_out_vel[1])
        response.past_behavior_ctrl_out_vel_z = float(self.past_behavior_ctrl_out_vel[2])
        
        response.behavior_ctrl_out_vel_x = float(behavior_ctrl_out_vel[0])
        response.behavior_ctrl_out_vel_y = float(behavior_ctrl_out_vel[1])
        response.behavior_ctrl_out_vel_z = float(behavior_ctrl_out_vel[2])

        
        #Pass next goal acceleration command (array)
        behavior_ctrl_out_accel = behavior_control_data[4]

        #Pass goal acceleration data to msg response
        response.behavior_ctrl_out_accel_x = float(behavior_ctrl_out_accel[0])
        response.behavior_ctrl_out_accel_y = float(behavior_ctrl_out_accel[1])
        response.behavior_ctrl_out_accel_z = float(behavior_ctrl_out_accel[2])

        #Pass next goal position
        behavior_ctrl_out_pos = behavior_control_data[3]

        #Pass goal position data to msg response
        response.behavior_ctrl_out_pos_x = float(behavior_ctrl_out_pos[0])
        response.behavior_ctrl_out_pos_y = float(behavior_ctrl_out_pos[1])
        response.behavior_ctrl_out_pos_z = float(behavior_ctrl_out_pos[2])

        #Pass current iteration results for the next iteration of calculations
        self.past_behavior_ctrl_out_vel = behavior_ctrl_out_vel

        return response



    def cmd_fwd_listener_callback(self, msg):
        
        self.swarm_waypoints = msg.swarm_waypoints
        self.formation_array = msg.formation_arg
        self.form_spacing_array = msg.formation_spacing
        self.form_node_size = msg.form_node_size
        # self.get_logger().info(f"self.form_node_size:  {self.form_node_size} ")
        self.ignore_drone_id_array = msg.ignore_drone_id_array
        # self.get_logger().info(f"self.ignore_drone_id_array:  {self.ignore_drone_id_array} ")
        #Pass relevant formation id information for this specific drone identified by "drone_id"
        self.form_assign_node_id = int(msg.drone_formation_id_array[self.drone_id - 1])
        # if (self.drone_id == 1 or self.drone_id == 2):
        #     self.form_assign_node_id = 1
        # self.get_logger().info(f"self.form_assign_node_id:  {self.form_assign_node_id} ")
    def behavior_controller_calc(self):
        
        #self.get_logger().info('behavior_controller_calc...')
        
        #CALL SERVICES FOR NECESSARY DATA

        #import Drone Swarm data as client/
        drone_swarm_data = self.drone_com_in_service()

        #import IMU data as client/ convert IMU data WRT inertial frame
        imu_data = self.call_imu_service()
        #self.get_logger().info(f'IMU service data: {imu_data}')
        
        #import GPS data as client/ convert IMU data WRT inertial frame
        gps_data = self.call_gps_service()
        #self.get_logger().info(f'GPS service data: {gps_data}')

        #import timestamp
        clock_data = self.call_sim_clock_service()
        #self.get_logger().info(f'GPS service data: {clock_data}')

        #import velocity
        velocity_data = self.call_vel_calc_service()
        

        #//////////////////////////////////////////////////////////////////

        #PASS SERVICE DATA INTO USEFUL VARIABLES/FORMAT

        #Pass drone swarm data
        swarm_size = drone_swarm_data.swarm_size #int
        for i in range(swarm_size):
            drone_id = int(i+1)
            index = int(i)
            arr_index = int((i)*3) #index every 3rd element to pull cartesian data of each drone (ie x1,y1,z1, x2,y2,y3)
            self.drone_swarm_dictionary['timestamp_'+f'{drone_id}'] = drone_swarm_data.timestamp[index]
            self.drone_swarm_dictionary['velocity_'+f'{drone_id}'] = drone_swarm_data.velocity[arr_index:(arr_index+3)]
            self.drone_swarm_dictionary['position_'+f'{drone_id}'] = drone_swarm_data.position[arr_index:(arr_index+3)]
        
        # self.get_logger().info(f'drone swarm dictionary data from DroneIntercomIn service: {self.drone_swarm_dictionary}')
        
        
        #Pass timestamp data
        behavior_timestamp = clock_data.sim_timestamp

        
        #Pass imu data
        # self.drone_lin_vel_bf[0] = imu_data.drone_lin_vel_x
        # self.drone_lin_vel_bf[1] = imu_data.drone_lin_vel_y
        # self.drone_lin_vel_bf[2] = imu_data.drone_lin_vel_z
        # self.drone_lin_accel_bf[0] = imu_data.drone_lin_accel_x
        # self.drone_lin_accel_bf[1] = imu_data.drone_lin_accel_y
        # self.drone_lin_accel_bf[2] = imu_data.drone_lin_accel_z
        self.drone_orient[0] = imu_data.drone_orient_x
        self.drone_orient[1] = imu_data.drone_orient_y
        self.drone_orient[2] = imu_data.drone_orient_z
        self.drone_orient[3] = imu_data.drone_orient_w
        
        drone_ang_vel_bf = np.zeros([3, 1])
        drone_ang_vel_bf[0] = imu_data.drone_ang_vel_x
        drone_ang_vel_bf[1] = imu_data.drone_ang_vel_y
        drone_ang_vel_bf[2] = imu_data.drone_ang_vel_z

        #Pass gps data
        self.drone_pos_if[0] = gps_data.de #x-axis
        self.drone_pos_if[1] = gps_data.dn #y-axis
        self.drone_pos_if[2] = gps_data.du #z-axis

        #Pass velocity data
        self.drone_lin_vel_if = np.array([[velocity_data.drone_vel_if[0]], [velocity_data.drone_vel_if[1]], [velocity_data.drone_vel_if[2]]])
        
        #Position initialized to none to get GPS data as starting position. After past position is estimated from behavior
        if (np.any(self.past_behavior_ctrl_out_pos == None)): #On fist iteration is null "scalar", and after is a (1,3) array, so for if statement to work, must compare all cells to "None"
            self.past_behavior_ctrl_out_pos = self.drone_pos_if

        #Convert imu quarternion orientation to euler angles (radians)
        self.drone_orient_euler = self.euler_from_quaternion(self.drone_orient[0], self.drone_orient[1], self.drone_orient[2], self.drone_orient[3])

        #Convert imu local velocity and acceleration from body frame to inertial frame
        
        self.drone_lin_accel_if = self.Rot_bf_to_if(self.drone_lin_accel_bf, self.drone_orient_euler[0], self.drone_orient_euler[1], self.drone_orient_euler[2])
        self.drone_lin_accel_if[2] = self.drone_lin_accel_if[2] - self.GRAVITY
        
        #////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        #GET GOAL POSITIONS AND NEXT FORMATION
        # Get average position of all drones in the world frame.
        # avg_swarm_pos = np.array([[-4],[-4],[0]]) #NOTE: Set for testing only
        form_node_size = self.form_node_size
        ignore_drone_id_array = self.ignore_drone_id_array
        form_assign_node_id = self.form_assign_node_id
        avg_swarm_pos = self.swarm_center_calc(form_node_size, ignore_drone_id_array, self.swarm_size, self.drone_swarm_dictionary) #NOTE: SET TO ZERO FOR TESTING
        # self.get_logger().info(f"avg_swarm_pos:  {avg_swarm_pos} ")
        #Set formation, spacing and goal position from the array data
        self.formation = self.formation_array[self.waypoint_count]
        self.form_spacing = self.form_spacing_array[self.waypoint_count]
        self.BOUND_AVOID_UAV = self.form_spacing*0.75 #set collision avoidance boundary based on swarm spacing
        index = self.waypoint_count*3
        swarm_goal_pos = np.array([[self.swarm_waypoints[index]], [self.swarm_waypoints[index+1]], [self.swarm_waypoints[index+2]]])
        
        #Check if this goal position is the final goal point (return true if the last goal point, else false)
        # last_goal_pos_bool = self.array_has_next_value(self.swarm_waypoints, index)

        #Get the distance from the current formation position to the goal/waypoint position
        form_to_goal_dist_xy =  self.dist_calc_2d(swarm_goal_pos, avg_swarm_pos)
        #If the formation is within the goal/waypoint boundary, then set the next point in the array as the goal   
        if (form_to_goal_dist_xy < self.BOUND_WAYPT and np.absolute(swarm_goal_pos[2] - avg_swarm_pos[2]) < self.MIN_BOUND_ALT):
            self.waypoint_count += 1
            swarm_goal_size = len(self.swarm_waypoints)/3
            if (self.waypoint_count <= swarm_goal_size):
                index = self.waypoint_count*3
                swarm_goal_pos = np.array([[self.swarm_waypoints[index]], [self.swarm_waypoints[index + 1]], [self.swarm_waypoints[index + 2]]])
                self.get_logger().info(f'NEW WAYPOINT, CURRENT POS: {swarm_goal_pos}')
                #Check if this goal position is the final goal point (return true if the last goal point, else false)
                # last_goal_pos_bool = self.array_has_next_value(self.swarm_waypoints, index)
                index = (self.waypoint_count-1)*3
                self.past_swarm_goal_pos = np.array([[self.swarm_waypoints[index]], [self.swarm_waypoints[index + 1]], [self.swarm_waypoints[index + 2]]])
                self.get_logger().info(f'NEW WAYPOINT, PAST POS: {self.past_swarm_goal_pos}')
                #Set bool for waypoint change
                # self.new_waypoint_bool = True
                # self.ascent_descent_goal_waypoint_pos = self.past_swarm_formation_at_waypoint_goal['form_uav'+ f'{form_assign_node_id}']
            else:
                self.get_logger().info(f'No new waypoint locations, final goal position achieved!!!')
            
            form_name_array_size = len(self.formation_array)
            if (self.waypoint_count <= form_name_array_size):
                self.formation = self.formation_array[self.waypoint_count]
                self.past_formation = self.formation_array[self.waypoint_count - 1]
            else:
                self.get_logger().info(f'No further formation commands...')

            form_space_array_size = len(self.form_spacing_array)
            if (self.waypoint_count <= form_space_array_size):
                self.form_spacing = self.form_spacing_array[self.waypoint_count]
            else:
                self.get_logger().info(f'No further formation spacing commands...')
        #////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        # BEHAVIORAL CONTROLLLER CALCULATIONS

        #NOTE: Maintain Altitude Behavior
        #Get unit vector direction in the inertial frame/ z-direction
        self.unit_vector_array[0, :] = np.array([[0.0, 0.0, (swarm_goal_pos[2] - self.drone_pos_if[2])/np.absolute(swarm_goal_pos[2] - self.drone_pos_if[2])]])

        #Determine the weight used for maintain altitude behavior (linearly adjusts)
        if (np.absolute(swarm_goal_pos[2] - self.drone_pos_if[2]) > self.BOUND_ALT): 
            self.f_weights[0,0] = self.W_ALT
        elif ((np.absolute(swarm_goal_pos[2] - self.drone_pos_if[2]) < self.MIN_BOUND_ALT)):
            self.f_weights[0,0] = 0.0
        else:
            self.f_weights[0,0] = self.W_ALT*np.divide(np.absolute(swarm_goal_pos[2] - self.drone_pos_if[2]), self.BOUND_ALT)
        

        
        #===========================================================================================================================
        #NOTE: Maintain Formation Behavior
        #NOTE: Get formation and assigned positions in formation
        # Get drone locations
        # Get self location
        # Calculate formation center based on lead and rear drones
        # Determine distance to formation position

        
        #------------------------------------------------------------------------------
        # Get average position of all drones in the world frame.
        # This will be used to determine where to place the formation center
        # avg_swarm_pos = self.swarm_center_calc(self.form_node_size, self.drone_swarm_dictionary)  #NOTE: Already calculated

        form_pos_dist_xy = None #NOTE: TEMPORARY FOR TROUBLESHOOTING. NOT NEEDED, BUT MUST COMMENT OUT CSV WRITE WHEN DONE
        # if (self.new_waypoint_bool == True): #Check to see if formation should rotate about it's axis
        #     self.new_waypoint_bool == False
        #     self.get_logger().info(f'NEW WAYPOINT, CHECK ANGLE: {self.form_rotate_bool}')
        #     self.form_rotate_bool = True
        #     self.rotation_axis_pos = self.past_swarm_goal_pos
        #     #----------------------------------------------------------------
        #     form_lead_vector = self.swarm_formation['form_uav'+ f'{1}'] - avg_swarm_pos
        #     form_lead_vector = form_lead_vector/(np.sqrt((form_lead_vector[0]**2)+(form_lead_vector[1]**2)+(form_lead_vector[2]**2)))
        #     form_lead_vector_row =  np.array([form_lead_vector[0,0], form_lead_vector[1,0], form_lead_vector[2,0]])
        #     #----------------------------------------------------------------
        #     #get normalized vector to the goal point from the drone swarm center
        #     swarm_goal_loc_row =  np.array([swarm_goal_pos[0,0] - avg_swarm_pos[0,0], swarm_goal_pos[1,0] - avg_swarm_pos[1,0], 0.0 - avg_swarm_pos[2,0]])
        #     swarm_goal_loc_row = swarm_goal_loc_row/(np.sqrt((swarm_goal_loc_row[0]**2)+(swarm_goal_loc_row[1]**2)+(swarm_goal_loc_row[2]**2)))
        #     #get the angle between the line formation and the goal point
        #     form2goal_angle = self.vector_angle_2d_swarm_form(form_lead_vector_row, swarm_goal_loc_row)
        #     #----------------------------------------------------------------
        #     formation_radius = ((form_lead_vector[0]**2)+(form_lead_vector[1]**2))**0.5
        #     rotation_increment = (0.25*3.8)/formation_radius
        #     #----------------------------------------------------------------
        #     if (form2goal_angle <= rotation_increment):
        #         self.form_rotate_bool = False
        #         self.get_logger().info(f'NEW WAYPOINT, BUT NEARLY STRAIGHT: {self.form_rotate_bool}')

        #NOTE: PROBLEMS IN ASIGNING FORMATION POSITIONS WITH FAILURE OF SINGLE DRONE. dUE TO THE FACT THAT PREVIOUSLY THE FORMATION HAD 3 POSITIONS AND THEN LOST 1 BEFORE REASSIGNMENT OF NEW FORMATION
        
        if (form_assign_node_id != None and form_assign_node_id != 0 and form_node_size > 1 and self.swarm_size > 1 and self.drone_id != np.any(ignore_drone_id_array) ):# and (np.absolute(swarm_goal_pos[2] - self.drone_pos_if[2]) < self.BOUND_ALT)): #Check if formation assignment has been passed to know which location would be the goal position in the formation
            # swarm_goal_dist_xy =  self.dist_calc_2d(swarm_goal_pos, avg_swarm_pos)
            # self.get_logger().info(f"SWARM GOAL DISTANCE  {swarm_goal_dist_xy} ")
            # if (np.absolute(swarm_goal_dist_xy) > 1):
            if (self.form_rotate_bool == True):
                self.get_logger().info(f'swarm_formation_bool: {self.form_rotate_bool}')
                #GET FORMATION POSITIONS IN WORLD FRAME ORIENTED TOWARDS GOAL POSITION
                if (self.drone_swarm_dictionary['timestamp_'+ f'{1}'] != None): #Check to ensure that swarm data has been received/initialized
                    
                    form_lead_vector = self.swarm_formation['form_uav'+ f'{1}'] - self.rotation_axis_pos #translate only the 1st position/drone_id=1 of the drone formation
                    formation_radius = ((form_lead_vector[0]**2)+(form_lead_vector[1]**2))**0.5
                    rotation_increment = (0.25*3.8)/formation_radius
                    
                    if (self.past_formation == FORMATION_CONST['formation_names'][0]): #Single File past_formation
                        # self.get_logger().info(f'swarm_formation_rotate 1: {self.swarm_formation_rotate}')

                        [self.swarm_formation, rotation_complete_bool] = self.rotate_formation_single_file(form_node_size, self.form_spacing, rotation_increment, self.past_swarm_goal_pos, form_lead_vector, swarm_goal_pos)
                        # self.get_logger().info(f'swarm_formation_rotate 2: {self.swarm_formation_rotate}')
                        if (rotation_complete_bool == True):
                            self.form_rotate_bool = False

                    elif (self.past_formation == FORMATION_CONST['formation_names'][1]):
                        self.swarm_formation = self.create_single_row_form(form_node_size, self.form_spacing, swarm_goal_pos, avg_swarm_pos)
                    elif (self.past_formation == FORMATION_CONST['formation_names'][3]):
                        self.swarm_formation = self.create_circle_form(form_node_size, self.form_spacing, swarm_goal_pos, avg_swarm_pos)
                    else:
                        self.get_logger().info(f'Could not match formation parameter, should be from the following selection: {FORMATION_CONST["formation_names"]}')
                        return
                #------------------------------------------------------------------------------
                self.goal_form_pos = (self.swarm_formation['form_uav'+ f'{form_assign_node_id}']) #pass column numpy array of goal formation position
                # else:
                #     self.goal_form_pos = self.past_swarm_formation_at_waypoint_goal['form_uav'+ f'{form_assign_node_id}']
                #     self.get_logger().info(f"Passing goal point formation...")
                # self.get_logger().info(f"SWARM FORMATION 6 (passed to goal_form_pos)  {self.swarm_formation_rotate} ")
                #Get unit vector direction in the inertial frame/ z-direction
                form_pos_dist_xy =  self.dist_calc_2d(self.goal_form_pos, self.drone_pos_if) #get distance from drone to goal formation position
                self.unit_vector_array[2, :] = np.array([[(self.goal_form_pos[0] - self.drone_pos_if[0])/form_pos_dist_xy , (self.goal_form_pos[1] - self.drone_pos_if[1])/form_pos_dist_xy , 0.0]])
                #------------------------------------------------------------------------------
                #Determine the weight used for maintain goal behavior (linearly adjusts)
                if (form_pos_dist_xy < self.MIN_BOUND_FORM or (np.absolute(swarm_goal_pos[2] - self.drone_pos_if[2]) > self.BOUND_ALT)):
                    self.f_weights[0,2] = 0.0
                elif (form_pos_dist_xy > self.BOUND_FORM):
                    self.f_weights[0,2] = self.W_FORM
                else:
                    self.f_weights[0,2] = self.W_FORM*np.divide(form_pos_dist_xy, self.BOUND_FORM)
                

            else:
                #GET FORMATION POSITIONS IN WORLD FRAME ORIENTED TOWARDS GOAL POSITION
                if (self.drone_swarm_dictionary['timestamp_'+ f'{1}'] != None): #Check to ensure that swarm data has been received/initialized
                    if (self.formation == FORMATION_CONST['formation_names'][0]): #Single File Formation
                        self.swarm_formation = self.create_single_file_form(form_node_size, self.form_spacing, swarm_goal_pos, avg_swarm_pos)
                        # self.get_logger().info(f"SWARM FORMATION 5 (passed to single file)  {self.swarm_formation} ")
                    elif (self.formation == FORMATION_CONST['formation_names'][1]):
                        self.swarm_formation = self.create_single_row_form(form_node_size, self.form_spacing, swarm_goal_pos, avg_swarm_pos)
                    elif (self.formation == FORMATION_CONST['formation_names'][3]):
                        self.swarm_formation = self.create_circle_form(form_node_size, self.form_spacing, swarm_goal_pos, avg_swarm_pos)
                    else:
                        self.get_logger().info(f'Could not match formation parameter, should be from the following selection: {FORMATION_CONST["formation_names"]}')
                        return
                #------------------------------------------------------------------------------
                self.goal_form_pos = self.swarm_formation['form_uav'+ f'{form_assign_node_id}'] #pass column numpy array of goal formation position
                # else:
                #     self.goal_form_pos = self.past_swarm_formation_at_waypoint_goal['form_uav'+ f'{form_assign_node_id}']
                #     self.get_logger().info(f"Passing goal point formation...")
                # self.get_logger().info(f"SWARM FORMATION 6 (passed to goal_form_pos)  {self.swarm_formation} ")
                #Get unit vector direction in the inertial frame/ z-direction
                form_pos_dist_xy =  self.dist_calc_2d(self.goal_form_pos, self.drone_pos_if) #get distance from drone to goal formation position
                self.unit_vector_array[2, :] = np.array([[(self.goal_form_pos[0] - self.drone_pos_if[0])/form_pos_dist_xy , (self.goal_form_pos[1] - self.drone_pos_if[1])/form_pos_dist_xy , 0.0]])
                #------------------------------------------------------------------------------
                #Determine the weight used for maintain goal behavior (linearly adjusts)
                if (form_pos_dist_xy < self.MIN_BOUND_FORM or (np.absolute(swarm_goal_pos[2] - self.drone_pos_if[2]) > self.BOUND_ALT)):
                    self.f_weights[0,2] = 0.0
                elif (form_pos_dist_xy > self.BOUND_FORM):
                    self.f_weights[0,2] = self.W_FORM
                else:
                    self.f_weights[0,2] = self.W_FORM*np.divide(form_pos_dist_xy, self.BOUND_FORM)
            
        else:
            # self.get_logger().info(f'FORMATION BEHAVIOR NOT ACTIVE...')
            self.unit_vector_array[2, :] = np.zeros((1,3))
            self.f_weights[0,2] = 0.0
        
        #===========================================================================================================================
    
        #NOTE: GOAL BEHAVIOR
        if (form_assign_node_id != None  and form_assign_node_id != 0 and self.swarm_size > 1 and (np.absolute(swarm_goal_pos[2] - self.drone_pos_if[2]) < self.BOUND_ALT)): #Check if formation assignment has been passed to know if the formation behavior has been processed as well to give the formation
            swarm_goal_dist_xy =  self.dist_calc_2d(swarm_goal_pos, avg_swarm_pos)
            if (np.absolute(swarm_goal_dist_xy) > 5): #Set condition to not recalculate positions once near the goal position
                # self.get_logger().info(f"SWARM FORMATION 8 (before getting swarm_formation_at_waypoint)  {self.swarm_formation} ")
                swarm_formation = self.swarm_formation.copy() #pass to dictionary copy to give to function. Dictionary will get udated unless a copy is given
                #Get translation vector from drone swarm position to the goal
                tranlation_vector = swarm_goal_pos - avg_swarm_pos
                #Translate formation center to the drone swarm center
                swarm_formation_at_waypoint_goal = self.translate_formation(form_node_size, swarm_formation, tranlation_vector)
                #Extract the formation node position assigned to the current drone
                goal_waypoint_pos = swarm_formation_at_waypoint_goal['form_uav'+ f'{form_assign_node_id}']
                # self.get_logger().info(f"SWARM FORMATION 8 (after getting swarm_formation_at_waypoint)  {self.swarm_formation} ")
                self.past_swarm_formation_at_waypoint_goal = swarm_formation_at_waypoint_goal.copy()
            else:
                goal_waypoint_pos = self.past_swarm_formation_at_waypoint_goal['form_uav'+ f'{form_assign_node_id}']
                swarm_formation_at_waypoint_goal = self.past_swarm_formation_at_waypoint_goal #NOTE: Pass for CSV fie only
            #Get unit vector direction in the inertial frame/ z-direction
            goal_dist_xy =  self.dist_calc_2d(goal_waypoint_pos, self.drone_pos_if) #get distance from drone to goal position
            self.unit_vector_array[1, :] = np.array([[(goal_waypoint_pos[0] - self.drone_pos_if[0])/goal_dist_xy , (goal_waypoint_pos[1] - self.drone_pos_if[1])/goal_dist_xy , 0.0]])

            #Determine the weight used for maintain goal behavior (linearly adjusts)
            if (self.form_rotate_bool == True): #if formation rotating, do not move towards goal
                self.f_weights[0,1] = 0.0
            else:
                if (goal_dist_xy < self.MIN_BOUND_GOAL or (np.absolute(swarm_goal_pos[2] - self.drone_pos_if[2]) > self.MIN_BOUND_ALT)):# and last_goal_pos_bool == True):
                    self.f_weights[0,1] = 0.0
                elif (goal_dist_xy > self.BOUND_GOAL):
                    self.f_weights[0,1] = self.W_GOAL
                    #self.get_logger().info(f'DRONE_{self.drone_id} weight meets last_goal_pos_bool = true...')
                # elif (last_goal_pos_bool == False):
                #     f_weight_goal = self.W_GOAL*np.divide(goal_dist_xy, self.BOUND_GOAL)
                #     self.get_logger().info(f'DRONE_{self.drone_id} weight meets last_goal_pos_bool = false...')
                #     if (f_weight_goal <= self.MIN_WAYPOINT_BOUND):
                #         f_weight_goal = self.W_GOAL*self.MIN_WAYPOINT_BOUND
                #         self.get_logger().info(f'DRONE_{self.drone_id} weight within MIN_WAYPOINT_BOUND...')
                #     self.get_logger().info(f'DRONE_{self.drone_id} weight: {f_weight_goal}')
                #     self.f_weights[0,1] = f_weight_goal
                        
                else:
                    # self.f_weights[0,1] = self.W_GOAL*np.divide(goal_dist_xy, self.BOUND_GOAL)
                    f_weight_goal = self.W_GOAL*np.divide(goal_dist_xy, self.BOUND_GOAL)
                    # self.get_logger().info(f'DRONE_{self.drone_id} weight meets last_goal_pos_bool = false...')
                    if (f_weight_goal <= self.W_GOAL*self.MIN_WAYPOINT_BOUND):
                        f_weight_goal = self.W_GOAL*self.MIN_WAYPOINT_BOUND #Set minimum speed to 50% instead of linearly decreasing further
                        # self.get_logger().info(f'DRONE_{self.drone_id} weight within MIN_WAYPOINT_BOUND...')
                    # self.get_logger().info(f'DRONE_{self.drone_id} weight: {f_weight_goal}')
                    self.f_weights[0,1] = f_weight_goal
        elif(self.swarm_size == 1):
            

            #Get unit vector direction in the inertial frame/ z-direction
            goal_dist_xy =  self.dist_calc_2d(swarm_goal_pos, self.drone_pos_if) #get distance from drone to goal position
            self.unit_vector_array[1, :] = np.array([[(swarm_goal_pos[0] - self.drone_pos_if[0])/goal_dist_xy , (swarm_goal_pos[1] - self.drone_pos_if[1])/goal_dist_xy , 0.0]])

            if (goal_dist_xy < self.MIN_BOUND_GOAL or (np.absolute(swarm_goal_pos[2] - self.drone_pos_if[2]) > self.MIN_BOUND_ALT)):# and last_goal_pos_bool == True):
                    self.f_weights[0,1] = 0.0
            elif (goal_dist_xy > self.BOUND_GOAL):
                self.f_weights[0,1] = self.W_GOAL                    
            else:
                # self.f_weights[0,1] = self.W_GOAL*np.divide(goal_dist_xy, self.BOUND_GOAL)
                f_weight_goal = self.W_GOAL*np.divide(goal_dist_xy, self.BOUND_GOAL)
                # self.get_logger().info(f'DRONE_{self.drone_id} weight meets last_goal_pos_bool = false...')
                if (f_weight_goal <= self.W_GOAL*self.MIN_WAYPOINT_BOUND):
                    f_weight_goal = self.W_GOAL*self.MIN_WAYPOINT_BOUND #Set minimum speed to 50% instead of linearly decreasing further
                    # self.get_logger().info(f'DRONE_{self.drone_id} weight within MIN_WAYPOINT_BOUND...')
                # self.get_logger().info(f'DRONE_{self.drone_id} weight: {f_weight_goal}')
                self.f_weights[0,1] = f_weight_goal
        else:
            # self.get_logger().info(f'GOAL BEHAVIOR NOT ACTIVE...')
            self.unit_vector_array[1, :] = np.zeros((1,3))
            self.f_weights[0,1] = 0.0
        #==========================================================================================================================
        #NOTE: Maintain Ascent/Descent Position Behavior
        

        # if (self.new_waypoint_bool == True):
        #     self.new_waypoint_bool = False
        #     self.ascent_descent_bool = True     
        #     self.get_logger().info(f'NEW WAYPOINT: {self.ascent_descent_bool}')

        if (self.ascent_descent_bool == True):
            drone_goal_dist_xy =  self.dist_calc_2d(self.ascent_descent_goal_waypoint_pos, self.drone_pos_if)#get distance from drone to goal position
            self.get_logger().info(f'ASCENT/DESCENT DRONE_GOAL_DIST: {drone_goal_dist_xy}')
            self.get_logger().info(f'ASCENT/DESCENT DRONE_GOAL POS: {self.ascent_descent_goal_waypoint_pos}')
            self.get_logger().info(f'ASCENT/DESCENT DRONE POS: {self.drone_pos_if}')
            #Get unit vector direction in the inertial frame/ z-direction
            self.unit_vector_array[4, :] = np.array([[(self.ascent_descent_goal_waypoint_pos[0] - self.drone_pos_if[0])/drone_goal_dist_xy , (self.ascent_descent_goal_waypoint_pos[1] - self.drone_pos_if[1])/drone_goal_dist_xy , 0.0]])
            self.get_logger().info(f'ASCENT/DESCENT unit vector: {self.unit_vector_array[4, :]}')
            
            # ratio_goal_pos_climb = ((self.BOUND_CLIMB - drone_goal_dist_xy)/(self.BOUND_CLIMB - self.MIN_BOUND_CLIMB))
        
            #Determine the weight used for maintain goal behavior (linearly adjusts)

            if (drone_goal_dist_xy < self.MIN_BOUND_CLIMB ):
                self.current_alt = self.drone_pos_if[2]
                self.f_weights[0,4] = 0.0
            elif (drone_goal_dist_xy > self.BOUND_CLIMB):
                self.f_weights[0,4] = self.W_CLIMB  
                self.f_weights[0,0] = 0.5 #altitude control set to zero 
                if (self.current_alt != self.drone_pos_if[2]):
                    self.unit_vector_array[0, :] = np.array([[0.0, 0.0,(self.current_alt - self.drone_pos_if[2])/np.absolute(self.current_alt - self.drone_pos_if[2])]])
                else:
                    self.unit_vector_array[0, :] = np.array([[0.0, 0.0, 0.0]])
            else:
                
                # self.f_weights[0,4] = self.W_CLIMB*np.divide(goal_dist_xy, self.BOUND_GOAL)
                f_weight_goal = self.W_CLIMB*np.divide(drone_goal_dist_xy, self.BOUND_CLIMB)
                # self.get_logger().info(f'DRONE_{self.drone_id} weight meets last_goal_pos_bool = false...')
                    # self.get_logger().info(f'DRONE_{self.drone_id} weight within MIN_WAYPOINT_BOUND...')
                # self.get_logger().info(f'DRONE_{self.drone_id} weight: {f_weight_goal}')
                self.f_weights[0,4] = f_weight_goal

                # self.f_weights[0,0] = self.W_ALT*ratio_goal_pos_climb # REDUCE ALTITUDE BEHAVIOR TO CONTROL POSITION
                self.f_weights[0,0] = 0.5 #altitude control set to zero 
                if (self.current_alt != self.drone_pos_if[2]):
                    self.unit_vector_array[0, :] = np.array([[0.0, 0.0,(self.current_alt - self.drone_pos_if[2])/np.absolute(self.current_alt - self.drone_pos_if[2])]])
                else:
                    self.unit_vector_array[0, :] = np.array([[0.0, 0.0, 0.0]])

            if ((np.absolute(swarm_goal_pos[2] - self.drone_pos_if[2]) < self.MIN_BOUND_ALT)):
                self.ascent_descent_bool = False
                self.get_logger().info(f'ASCENT/DESCENT COMPLETE: {self.ascent_descent_bool}')
        else:
            self.f_weights[0,4] = 0.0
            self.unit_vector_array[4, :] = np.zeros((1,3))
        #===========================================================================================================================


        #NOTE: UAV COLLISION AVOIDANCE VELOCITY-ALGORITHM
        f_weight_avoid = np.zeros((1,self.swarm_size))
        unit_vector_avoid = np.zeros((3,self.swarm_size))
        max_ratio_avoidance = 0.0
        if (form_assign_node_id != None and (np.absolute(swarm_goal_pos[2] - self.drone_pos_if[2]) < self.BOUND_ALT) and self.swarm_size > 1): #Check if formation assignment has been passed to know if the formation behavior has been processed as well to give the formation
            
            vect_avoidance_sum = np.zeros((3,1), dtype=float)
            
            collision_count = 0 #count the number of drones within the collision boundary

            for i in range(swarm_size):
                drone_id = int(i+1)
                # if (drone_id != np.any(ignore_drone_id_array)): #ignore collision calc for faulty drones

                index = int(i)
                # self.get_logger().info(f'DRONE_{self.drone_id} position: {self.drone_pos_if}')
                # self.get_logger().info(f'DRONE_{drone_id} position: {self.drone_swarm_dictionary["position_"+f"{drone_id}"]}')
                drone2drone_dist_xy =  self.dist_calc_2d(self.drone_swarm_dictionary['position_'+f'{drone_id}'], self.drone_pos_if)
                #Consider 3d distance to drone to prevent calculation with faulty drones below goal altitude
                drone2drone_dist_xyz =  self.dist_calc_3d(self.drone_swarm_dictionary['position_'+f'{drone_id}'], self.drone_pos_if)
                
                # self.get_logger().info(f'DRONE_{self.drone_id} to DRONE_{drone_id} distance: {drone2drone_dist_xyz}')
                

                v_self_norm = (np.asscalar(self.drone_lin_vel_if[0])**2 + np.asscalar(self.drone_lin_vel_if[1])**2)**0.5
                v_other_norm = ((self.drone_swarm_dictionary['velocity_'+f'{drone_id}'][0])**2 + (self.drone_swarm_dictionary['velocity_'+f'{drone_id}'][1])**2)**0.5
                collision_bool = False
                if ( drone2drone_dist_xyz <= self.BOUND_AVOID_UAV and drone_id != self.drone_id and (v_self_norm > 1*(10**(-4)) or v_other_norm > 1*(10**(-4)) )):
                    #using this method causes collisions at steep collision angles between drones
                    # v_collision_column_array = self.collision_avoidance_vector_calc(self.drone_lin_vel_if , self.drone_swarm_dictionary['velocity_'+f'{drone_id}'])
                    [collision_dt, collision_bool] = self.interagent_time2collision(self.COLLISION_BOUND_AVOID_UAV , self.drone_lin_vel_if, self.drone_pos_if,  self.drone_swarm_dictionary['velocity_'+f'{drone_id}'], self.drone_swarm_dictionary['position_'+f'{drone_id}'])
                    
                    v_normal_collision_column_array = np.vstack([
                        [-1*np.asscalar(self.drone_swarm_dictionary['position_'+f'{drone_id}'][0] - self.drone_pos_if[0])/drone2drone_dist_xy] , 
                        [-1*np.asscalar(self.drone_swarm_dictionary['position_'+f'{drone_id}'][1] - self.drone_pos_if[1])/drone2drone_dist_xy] , 
                        [0.0]
                    ])
                    
                    v_drone2drone = (self.drone_swarm_dictionary['position_'+f'{drone_id}'] - self.drone_pos_if)/drone2drone_dist_xy
                    v_tangent_collision_column_array = np.vstack([
                        [-1*np.copysign(np.asscalar(v_drone2drone[1,0]), self.unit_vector_array[2, 0])] , 
                        [-1*np.copysign(np.asscalar(v_drone2drone[0,0]), self.unit_vector_array[2, 1])] , 
                        [0.0]
                    ])

                    v_collision_column_array = v_normal_collision_column_array + v_tangent_collision_column_array
                    unit_vector_avoid[:,index]= v_collision_column_array[:,0]
                    #CHECK IF COLLISION AVOIDANCE VECTOR, FORMATION VECTOR ARE NEARLY COLINEAR. IF SO, MAKE AVOIDANCE VECTOR NORMAL
                    # if ((self.unit_vector_array[2, 0]) != 0.0 and (self.unit_vector_array[2, 1]) != 0.0): #ensure formation vector isnt zero
                    #     # self.get_logger().info(f'CHECKING FORM TO AVOID VECTOR ANGLE...')

                    #     v_collision_row_array = np.hstack([-1*(self.drone_swarm_dictionary['position_'+f'{drone_id}'][0] - self.drone_pos_if[0])/drone2drone_dist_xy,
                    #                                       -1*(self.drone_swarm_dictionary['position_'+f'{drone_id}'][1] - self.drone_pos_if[1])/drone2drone_dist_xy ,
                    #                                       0.0])
                    #     # Ensure Vref_x and Vb are 1-dimensional arrays (vectors) for the cross product operation
                    #     v_ref_x = self.unit_vector_array[2, :] #.flatten()
                    #     # self.get_logger().info(f'FORM UNIT VECTOR{v_ref_x}')
                    #     v_b = v_collision_row_array
                    #     # self.get_logger().info(f'COLLISION AVOIDANCE UNIT VECTOR {v_b}')
                    #     angle_rad = self.vector_angle_2d_swarm_form(v_ref_x, v_b)
                    #     angle_deg = 180.0 - np.absolute(angle_rad*(180.0/np.pi))
                    #     if (np.absolute(angle_deg) < 20): #and (v_self_norm < 0.2 or v_other_norm < 0.2)): #check if obstacle avoidance vector is less than the tolerance of 10 deg
                    #         v_collision_column_array = self.collision_avoidance_vector_calc(self.drone_lin_vel_if , self.drone_swarm_dictionary['velocity_'+f'{drone_id}'])
                    #         self.get_logger().info(f'FORM AND AVOID VECTOR NEARLY 180 DEG... NEW UNIT VECTOR {v_collision_column_array}')
                    #         self.get_logger().info(f'NEW UNIT VECTOR 180 deg{v_collision_column_array}')
                    #     elif (v_self_norm < 0.2 and v_other_norm < 0.2 and drone2drone_dist_xy < self.MIN_BOUND_AVOID_UAV):
                    #         # self.get_logger().info(f'DRONE_{self.drone_id} to DRONE_{drone_id} below 0.2m/s and must be avoided: {drone2drone_dist_xy}')
                    #         v_drone2drone = (self.drone_swarm_dictionary['position_'+f'{drone_id}'] - self.drone_pos_if)/drone2drone_dist_xy
                    #         v_collision_column_array = np.vstack([
                    #             [-1*np.copysign(np.asscalar(v_drone2drone[1,0]), self.unit_vector_array[2, 0])] , 
                    #             [-1*np.copysign(np.asscalar(v_drone2drone[0,0]), self.unit_vector_array[2, 1])] , 
                    #             [0.0]
                    #         ])
                    #         self.get_logger().info(f'NEW UNIT VECTOR 180 deg{v_collision_column_array}')
                    #         self.get_logger().info(f'NEW UNIT VECTOR, slow collision{v_collision_column_array}')
                        
                    #     v_collision_column_array = np.array(v_collision_column_array, copy=False)
                    #     self.get_logger().info(f'NEW UNIT VECTOR NUMPY {v_collision_column_array}')
                    # Convert v_collision_column_array elements to float using astype
                    v_collision_column_array = v_collision_column_array.astype(float)

                    if (collision_bool == True and collision_dt <= 3.0):
                        if (collision_dt <= 0.5):
                            collision_dt = 0.5 #sec

                        f_weight_avoid[0,index] = np.divide(self.W_AVOID_UAV, collision_dt)
                        # print('f_weight_avoid')
                        # print(f_weight_avoid)
                        # unit_vector_avoid[:,index]= v_collision_column_array.flatten()
                        # print('unit_vector_avoid')
                        # print(unit_vector_avoid)
                        # self.get_logger().info(f'DRONE_{self.drone_id} to DRONE_{drone_id} unit vector { unit_vector_avoid[:,index]} with weight {f_weight_avoid[0,index]}')
                        vect_avoidance_sum += v_collision_column_array*np.divide(self.W_AVOID_UAV, collision_dt)
                    
                        collision_count += 1 #increment count for number of drones in collision distance

                        if (self.BOUND_AVOID_UAV > 12.0):
                            ratio_bound = self.BOUND_AVOID_UAV
                        else:
                            # ratio_bound = 12.0
                            ratio_bound = self.BOUND_AVOID_UAV

                        ratio_avoidance = ((ratio_bound - drone2drone_dist_xy)/(self.BOUND_AVOID_UAV - self.MIN_BOUND_AVOID_UAV))
                    
                        if (ratio_avoidance > max_ratio_avoidance):
                            max_ratio_avoidance = ratio_avoidance
                            # self.get_logger().info(f'reduction ratio {max_ratio_avoidance}')
                
                if (drone2drone_dist_xyz <= self.MIN_BOUND_AVOID_UAV and collision_bool == True and drone_id != self.drone_id): #Check whether if the drones are so close to eachother that the formation behavior needs to be turned off
                    if (self.f_weights[0,1] >= 0.1):
                        self.f_weights[0,1] = 0.1 #Set goal behavior weight to zero
                    if ( self.f_weights[0,2] >= 0.1):
                        self.f_weights[0,2] = 0.1 #Set formation behavior to zero
                    # self.get_logger().info(f'DRONE_{self.drone_id} to DRONE_{drone_id} distance: {drone2drone_dist_xy} is below {self.MIN_BOUND_AVOID_UAV}')

                elif (max_ratio_avoidance > 0.0 and drone2drone_dist_xyz <= self.BOUND_AVOID_UAV and collision_bool == True and drone_id != self.drone_id):
                    reduction_ratio = max_ratio_avoidance
                    self.f_weights[0,1] = self.f_weights[0,1]*reduction_ratio #reduce goal behavior to equivalent ratio set for collision avoidance
                    self.f_weights[0,2] = self.f_weights[0,2]*reduction_ratio #reduce formation behavior to equivalent ratio set for collision avoidance
        
        else:
            # self.get_logger().info(f'UAV COLLISION AVOIDANCE BEHAVIOR NOT ACTIVE... 2')
            vect_avoidance_sum = np.zeros((3,1))

        #////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        # vect_avoidance_sum = np.zeros((3,1))
        #Determine velocity vector components (multiply weights with vectors and add all vectors together in the x,y,z directions respectively)
        vel_cmd_out = np.zeros([3, 1])
        vel_cmd_out[0] = np.dot(self.f_weights, self.unit_vector_array[:, 0]) + vect_avoidance_sum[0,0]
        vel_cmd_out[1] = np.dot(self.f_weights, self.unit_vector_array[:, 1]) + vect_avoidance_sum[1,0]
        vel_cmd_out[2] = np.dot(self.f_weights, self.unit_vector_array[:, 2]) + vect_avoidance_sum[2,0]

        #///////////////////////////////////
        vel_cmd_out_rescale = np.zeros([3, 1])
        if (np.any(vel_cmd_out) > 1): #NOTE: IF WEIGHTS SET TO BE A MAX OF 1, THEN THIS CONDITION IS UNNECESSARY

            #Normalize the velocity components and map to the full range of the quadrotor's full speed
            vel_cmd_out_rescale[0] = self.MAX_LIN_VEL_XY*(vel_cmd_out[0]/(np.sqrt((vel_cmd_out[0]**2)+(vel_cmd_out[1]**2)+(vel_cmd_out[2]**2))))
            vel_cmd_out_rescale[1] = self.MAX_LIN_VEL_XY*(vel_cmd_out[1]/(np.sqrt((vel_cmd_out[0]**2)+(vel_cmd_out[1]**2)+(vel_cmd_out[2]**2))))
            vel_cmd_out_rescale[2] = self.MAX_LIN_VEL_Z*(vel_cmd_out[2]/(np.sqrt((vel_cmd_out[0]**2)+(vel_cmd_out[1]**2)+(vel_cmd_out[2]**2))))
        else:
            vel_cmd_out_rescale[0] = self.MAX_LIN_VEL_XY*vel_cmd_out[0]
            vel_cmd_out_rescale[1] = self.MAX_LIN_VEL_XY*vel_cmd_out[1]
            vel_cmd_out_rescale[2] = self.MAX_LIN_VEL_Z*vel_cmd_out[2]
        
        #////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        
        #////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        
        #Determine the direction the quadrotor should face from the x-xis (Yaw angle)
        orient_unit_vector = vel_cmd_out/(np.sqrt((vel_cmd_out[0]**2)+(vel_cmd_out[1]**2)+(vel_cmd_out[2]**2)))

        orient_unit_vector_row = np.array([orient_unit_vector[0,0], orient_unit_vector[1,0], orient_unit_vector[2,0]]) #convert 1-d column vector to 1-d row vector
        
        vel_cmd_goal_yaw_if = self.vector_angle_2d(orient_unit_vector_row) #goal yaw angle in radians

        vel_cmd_goal_yaw_if = vel_cmd_goal_yaw_if - self.ANG_X_TO_C #Adjust total yaw angle rotation to ensure the front of the quadrotor faces the desired trajectory along its centerline offset from the positive x-axis
        
        #Convert IMU angular rates from the body frame to the inertial frame
        drone_ang_vel_if = self.Rot_bf_to_if(drone_ang_vel_bf, self.drone_orient_euler[0], self.drone_orient_euler[1], self.drone_orient_euler[2])
        
        ang_dist_yaw = vel_cmd_goal_yaw_if - self.drone_orient_euler[2]
        #Determine the weight used for maintain orientation behavior (linearly adjusts)
        if (ang_dist_yaw > self.BOUND_ANG_YAW): 
            yaw_ang_weight = self.W_YAW
        else:
            yaw_ang_weight = self.W_YAW*np.divide(ang_dist_yaw, self.BOUND_ANG_YAW)
        #self.get_logger().info(f'yaw_ang_weight: {yaw_ang_weight}')
        goal_ang_vel_psi_if = yaw_ang_weight*self.MAX_YAW_ANG_VEL
        #self.get_logger().info(f'goal_ang_vel_psi_if: {goal_ang_vel_psi_if}')
        goal_psi_y_if = ((goal_ang_vel_psi_if)*0.25) + self.drone_orient_euler[2] #NOTE: Assuming the drone only moves in a counter-clockwise direction
        #self.get_logger().info(f'self.CTRL_TIME_STEP: {self.CTRL_TIME_STEP}')
        #self.get_logger().info(f'self.drone_orient_euler[2]: {self.drone_orient_euler[2]}')
        #self.get_logger().info(f'goal_psi_y_if: {goal_psi_y_if}')

                
        #////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        
        #////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        

        #Calculate next goal position taking the calculated velocity, time step before the controller is called for its next iteration, and the current drone position
        behavior_ctrl_out_pos = (vel_cmd_out_rescale*1) + self.drone_pos_if #self.CTRL_TIME_STEP #get estimated point for a travel time of 1 second.
        behavior_ctrl_out_pos[1] = (vel_cmd_out_rescale[1]*self.CTRL_TIME_STEP) + self.drone_pos_if[1]
        behavior_ctrl_out_pos[0] = (vel_cmd_out_rescale[0]*self.CTRL_TIME_STEP) + self.drone_pos_if[0]
        # behavior_ctrl_out_pos Was previously used for comparison
        if (np.absolute(swarm_goal_pos[2] - self.drone_pos_if[2]) <= self.BOUND_ALT_POS_CTRL):
            behavior_ctrl_out_pos[2] = swarm_goal_pos[2]
        
        if (np.absolute(swarm_goal_pos[0] - self.drone_pos_if[0]) <= self.BOUND_GOAL):
            behavior_ctrl_out_pos[0] = swarm_goal_pos[0]
        
        if (np.absolute(swarm_goal_pos[1] - self.drone_pos_if[1]) <= self.BOUND_GOAL):
            behavior_ctrl_out_pos[1] = swarm_goal_pos[1]
        self.past_behavior_ctrl_out_pos = behavior_ctrl_out_pos
        
        #////////////////////////////////////

        #Calculate next acceleration goal command based on the past velocity command and the fixed/designated controller time step (not based on actual run-time)
        # behavior_ctrl_out_accel_2 = vel_cmd_out_rescale/4 #(vel_cmd_out_rescale - self.drone_lin_vel_if)#/self.CTRL_TIME_STEP #Give it 1 sec to achieve goal

        if ((behavior_timestamp - self.past_timestamp) > 0):
            behavior_ctrl_out_accel = (vel_cmd_out_rescale - self.past_behavior_ctrl_out_vel)/(behavior_timestamp - self.past_timestamp)
        else:
            behavior_ctrl_out_accel = np.zeros_like(vel_cmd_out_rescale)
            
        self.past_behavior_ctrl_out_vel = vel_cmd_out_rescale
        self.past_timestamp = behavior_timestamp
        # behavior_ctrl_out_accel_xy_scalar = (np.sqrt((behavior_ctrl_out_accel[0]**2)+(behavior_ctrl_out_accel[1]**2)))
        

        #self.get_logger().info(f'behavior_ctrl_out_accel (before max/min check): {behavior_ctrl_out_accel}')
        if ( behavior_ctrl_out_accel[2] > self.MAX_LIN_ACCEL_Z):
            behavior_ctrl_out_accel[2] = self.MAX_LIN_ACCEL_Z
            
        elif (behavior_ctrl_out_accel[2] < (-1*self.GRAVITY)):
            behavior_ctrl_out_accel[2] = (-1*self.GRAVITY)
        #self.get_logger().info(f'behavior_ctrl_out_accel (after max/min check): {behavior_ctrl_out_accel}')

        #////////////////////////////////////////////////////////////////////////////////
        #////////////////////////////////////////////////////////////////////////////////
        # csv_list_yaw = [behavior_timestamp,
        #                 self.drone_orient_euler[0,0], self.drone_orient_euler[1,0], self.drone_orient_euler[2,0],
        #                 vel_cmd_goal_yaw_if,
        #                 yaw_ang_weight,
        #                 goal_ang_vel_psi_if,
        #                 goal_psi_y_if ]
        # csv_list_header_yaw = [ "behavior_timestamp",
        #                         "drone_orient_euler[x]", "drone_orient_euler[y]", "drone_orient_euler[z]",  
        #                         "vel_cmd_goal_yaw_if",
        #                         "yaw_ang_weight",
        #                         "goal_ang_vel_psi_if",
        #                         "goal_psi_y_if" ]

        # directory_csv_yaw = os.path.join('/home/cnchano/', self.drone_namespace + '_behavior_yaw_out.csv') #os.path.join(os.getcwd(), 'src/drone_control/behavior_ctrl_out.csv')
        # # self.get_logger().info(f'Getting csv file path: {directory_csv}...')
        # if (self.iter_count == 0.0): #self.prev_ctrl_loop_timestamp is initialized at zero in the initialization function, therefore the first run of the script should have this value equal to zero
        #     # self.get_logger().info(f'creating Behavior Ctrl csv file...')
        #     with open(directory_csv_yaw, 'w', newline='') as f_object2:
        #         writer_object2 = writer(f_object2)
        #         writer_object2.writerow(csv_list_header_yaw)
        #         writer_object2.writerow(csv_list_yaw)
        #         # Close the file object
        #         f_object2.close()
                
        #     # self.get_logger().info(f'Finished first write to csv file...')
        # else:
        #     # Open our existing CSV file in append mode
        #     # Create a file object for this file
        #     with open(directory_csv_yaw, 'a') as f_object2:
            
        #         # Pass this file object to csv.writer()
        #         # and get a writer object
        #         writer_object2 = writer(f_object2)
            
        #         # Pass the list as an argument into
        #         # the writerow()
        #         writer_object2.writerow(csv_list_yaw)
            
        #         # Close the file object
        #         f_object2.close()
        
        # # List that we want to add as a new row for data collection
        # csv_list = [
        # behavior_timestamp,
        # #goal_dist_xy, #caused error since is not defined until if statement is fulfilled in goal behavior
        # self.drone_orient_euler[0,0], self.drone_orient_euler[1,0], self.drone_orient_euler[2,0],
        # self.drone_pos_if[0,0], self.drone_pos_if[1,0], self.drone_pos_if[2,0],
        # self.drone_lin_vel_if[0,0], self.drone_lin_vel_if[1,0], self.drone_lin_vel_if[2,0], 
        # self.drone_lin_vel_bf[0,0], self.drone_lin_vel_bf[1,0], self.drone_lin_vel_bf[2,0],
        # self.drone_orient[0,0], self.drone_orient[1,0], self.drone_orient[2,0], self.drone_orient[3,0],
        # self.drone_orient_euler[0,0], self.drone_orient_euler[1,0], self.drone_orient_euler[2,0],
        # vel_cmd_out[0,0], vel_cmd_out[1,0], vel_cmd_out[2,0],
        # vel_cmd_out_rescale[0,0], vel_cmd_out_rescale[1,0], vel_cmd_out_rescale[2,0],
        
        # goal_ang_vel_psi_if,
        # goal_psi_y_if,
        # vel_cmd_goal_yaw_if,
        # behavior_ctrl_out_accel[0,0], behavior_ctrl_out_accel[1,0], behavior_ctrl_out_accel[2,0],
        # # behavior_ctrl_out_accel_2[0,0], behavior_ctrl_out_accel_2[1,0], behavior_ctrl_out_accel_2[2,0],
        # behavior_ctrl_out_pos[0,0], behavior_ctrl_out_pos[1,0], behavior_ctrl_out_pos[2,0],
        # self.past_behavior_ctrl_out_pos[0,0], self.past_behavior_ctrl_out_pos[1,0], self.past_behavior_ctrl_out_pos[2,0],
        # self.unit_vector_array[0, 2],
        # ]

        # csv_list_header = [
        # "behavior_timestamp",
        # #"goal_dist_xy",
        # "drone_orient_euler[x]", "drone_orient_euler[y]", "drone_orient_euler[z]",
        # "GPS_PC_Pos_IF_X", "GPS_PC_Pos_IF_Y", "GPS_PC_Pos_IF_Z",
        # "drone_lin_vel_if[x]", "drone_lin_vel_if[y]", "drone_lin_vel_if[z]", 
        # "drone_lin_vel_bf[x]", "drone_lin_vel_bf[y]", "drone_lin_vel_bf[z]",
        # "drone_orient[x]", "drone_orient[y]", "drone_orient[z]", "drone_orient[w]",
        # "drone_orient_euler", "drone_orient_euler[y]", "drone_orient_euler[z]",
        # "vel_cmd_out[x]", "vel_cmd_out[y]", "vel_cmd_out[z]",
        # "vel_cmd_out_rescale[x]", "vel_cmd_out_rescale[y]", "vel_cmd_out_rescale[z]",
        # "goal_ang_vel_psi_if",
        # "goal_psi_y_if",
        # "vel_cmd_goal_yaw_if",
        # "behavior_ctrl_out_accel[0]", "behavior_ctrl_out_accel[1]", "behavior_ctrl_out_accel[2]",
        # # "2behavior_ctrl_out_accel[0]", "2behavior_ctrl_out_accel[1]", "2behavior_ctrl_out_accel[2]",
        # "behavior_ctrl_out_pos[x]", "behavior_ctrl_out_pos[y]", "behavior_ctrl_out_pos[z]",
        # "past_behavior_ctrl_out_pos[0]", "past_behavior_ctrl_out_pos[1]", "past_behavior_ctrl_out_pos[2]",
        # "unit_vector_array[0, 2]",
        # ]

        # directory_csv = os.path.join('/home/cnchano/', self.drone_namespace + '_behavior_ctrl_out.csv') #os.path.join(os.getcwd(), 'src/drone_control/behavior_ctrl_out.csv')
        # # self.get_logger().info(f'Getting csv file path: {directory_csv}...')
        # if (self.iter_count == 0.0): #self.prev_ctrl_loop_timestamp is initialized at zero in the initialization function, therefore the first run of the script should have this value equal to zero
        #     # self.get_logger().info(f'creating Behavior Ctrl csv file...')
        #     with open(directory_csv, 'w', newline='') as f_object:
        #         writer_object = writer(f_object)
        #         writer_object.writerow(csv_list_header)
        #         writer_object.writerow(csv_list)
        #         # Close the file object
        #         f_object.close()
                
                
        #     # self.get_logger().info(f'Finished first write to csv file...')
        # else:
        #     # Open our existing CSV file in append mode
        #     # Create a file object for this file
        #     with open(directory_csv, 'a') as f_object:
            
        #         # Pass this file object to csv.writer()
        #         # and get a writer object
        #         writer_object = writer(f_object)
            
        #         # Pass the list as an argument into
        #         # the writerow()
        #         writer_object.writerow(csv_list)
            
        #         # Close the file object
        #         f_object.close()

        # # List that we want to add as a new row for data collection
        # csv_list = [
        # behavior_timestamp,
        # self.drone_pos_if[0,0], self.drone_pos_if[1,0], self.drone_pos_if[2,0],
        # self.drone_lin_vel_if[0,0], self.drone_lin_vel_if[1,0], self.drone_lin_vel_if[2,0], 
        # self.f_weights[0,0], self.f_weights[0,1], self.f_weights[0,2], self.f_weights[0,3],
        # self.unit_vector_array[0, :], self.unit_vector_array[1, :], self.unit_vector_array[2, :], self.unit_vector_array[3, :], 
        
        # vel_cmd_out[0,0], vel_cmd_out[1,0], vel_cmd_out[2,0],
        # vel_cmd_out_rescale[0,0], vel_cmd_out_rescale[1,0], vel_cmd_out_rescale[2,0],
        
        
        # behavior_ctrl_out_accel[0,0], behavior_ctrl_out_accel[1,0], behavior_ctrl_out_accel[2,0],
        # # behavior_ctrl_out_accel_2[0,0], behavior_ctrl_out_accel_2[1,0], behavior_ctrl_out_accel_2[2,0],
        # behavior_ctrl_out_pos[0,0], behavior_ctrl_out_pos[1,0], behavior_ctrl_out_pos[2,0],
        

        # ]

        # csv_list_header = [
        # "behavior_timestamp",
        # "GPS_PC_Pos_IF_X", "GPS_PC_Pos_IF_Y", "GPS_PC_Pos_IF_Z",
        # "drone_lin_vel_if[x]", "drone_lin_vel_if[y]", "drone_lin_vel_if[z]", 
        # "f_weights_altitude", "f_weights_goal", "f_weights_formation", "f_weights_UAV_avoidance",
        # "unit_vector_array_altitude", "unit_vector_array_goal", "unit_vector_array_formation", "unit_vector_array_UAV_avoidance",

        # "vel_cmd_out[x]", "vel_cmd_out[y]", "vel_cmd_out[z]",
        # "vel_cmd_out_rescale[x]", "vel_cmd_out_rescale[y]", "vel_cmd_out_rescale[z]",
        
        # "behavior_ctrl_out_accel[0]", "behavior_ctrl_out_accel[1]", "behavior_ctrl_out_accel[2]",
        # # "2behavior_ctrl_out_accel[0]", "2behavior_ctrl_out_accel[1]", "2behavior_ctrl_out_accel[2]",
        # "behavior_ctrl_out_pos[x]", "behavior_ctrl_out_pos[y]", "behavior_ctrl_out_pos[z]",
        

        # ]

        # directory_csv = os.path.join('/home/cnchano/', self.drone_namespace + '_behaviors_calc.csv') #os.path.join(os.getcwd(), 'src/drone_control/behavior_ctrl_out.csv')
        # # self.get_logger().info(f'Getting csv file path: {directory_csv}...')
        # if (self.iter_count == 0.0): #self.prev_ctrl_loop_timestamp is initialized at zero in the initialization function, therefore the first run of the script should have this value equal to zero
        #     # self.get_logger().info(f'creating Behavior Ctrl csv file...')
        #     with open(directory_csv, 'w', newline='') as f_object3:
        #         writer_object = writer(f_object3)
        #         writer_object.writerow(csv_list_header)
        #         writer_object.writerow(csv_list)
        #         # Close the file object
        #         f_object3.close()
                
        #     # self.get_logger().info(f'Finished first write to csv file...')
        # else:
        #     # Open our existing CSV file in append mode
        #     # Create a file object for this file
        #     with open(directory_csv, 'a') as f_object3:
            
        #         # Pass this file object to csv.writer()
        #         # and get a writer object
        #         writer_object = writer(f_object3)
            
        #         # Pass the list as an argument into
        #         # the writerow()
        #         writer_object.writerow(csv_list)
            
        #         # Close the file object
        #         f_object3.close()

        #         # List that we want to add as a new row for data collection
        # csv_list_formation = [
        # behavior_timestamp,
        # self.drone_pos_if[0,0], self.drone_pos_if[1,0], self.drone_pos_if[2,0],
        # swarm_goal_pos[0,0], swarm_goal_pos[1,0], swarm_goal_pos[2,0], 

        # swarm_formation_at_waypoint_goal['form_uav'+ f'{1}'], swarm_formation_at_waypoint_goal['form_uav'+ f'{2}'], self.swarm_formation['form_uav'+ f'{3}'],
        # form_assign_node_id,

        # goal_waypoint_pos[0,0], goal_waypoint_pos[1,0], goal_waypoint_pos[2,0],
        # form_pos_dist_xy,

        # self.f_weights[0,0], self.f_weights[0,1], self.f_weights[0,2], self.f_weights[0,3],
        # self.unit_vector_array[0, :], self.unit_vector_array[1, :], self.unit_vector_array[2, :], self.unit_vector_array[3, :], 
        
        # vel_cmd_out[0,0], vel_cmd_out[1,0], vel_cmd_out[2,0],
        # vel_cmd_out_rescale[0,0], vel_cmd_out_rescale[1,0], vel_cmd_out_rescale[2,0],
        # ]
       
        # csv_list_header_formation = [
        # "behavior_timestamp",

        # "GPS_Pos_IF_X", "GPS_Pos_IF_Y", "GPS_Pos_IF_Z",
        # "Swarm_Goal_Pos_X", "Swarm_Goal_Pos_Y", "Swarm_Goal_Pos_Z", 

        # "Goal_Pos_UAV_1", "Goal_Pos_UAV_2", "Goal_Pos_UAV_3", 
        # "UAV_Form_Node_ID",

        # "Goal_UAV_GoalForm_Pos_X", "Goal_UAV_GoalForm_Pos_Y", "Goal_UAV_GoalForm_Pos_Z",
        # "form_pos_dist_xy",

        # "f_weights altitude", "f_weights goal", "f_weights formation", "f_weights UAV avoidance",
        # "unit_vector_array altitude", "unit_vector_array goal", "unit_vector_array formation", "unit_vector_array UAV avoidance",

        # "vel_cmd_out[x]", "vel_cmd_out[y]", "vel_cmd_out[z]",
        # "vel_cmd_out_rescale[x]", "vel_cmd_out_rescale[y]", "vel_cmd_out_rescale[z]",
        # ]

        # directory_csv = os.path.join('/home/cnchano/', self.drone_namespace + '_behavior_calc_goal.csv') #os.path.join(os.getcwd(), 'src/drone_control/behavior_ctrl_out.csv')
        # # self.get_logger().info(f'Getting csv file path: {directory_csv}...')
        # if (self.iter_count == 0.0): #self.prev_ctrl_loop_timestamp is initialized at zero in the initialization function, therefore the first run of the script should have this value equal to zero
        #     # self.get_logger().info(f'creating Behavior Ctrl csv file...')
        #     with open(directory_csv, 'w', newline='') as f_object4:
        #         writer_object = writer(f_object4)
        #         writer_object.writerow(csv_list_header_formation)
        #         writer_object.writerow(csv_list_formation)
        #         # Close the file object
        #         f_object4.close()
        #         # self.iter_count += 1
        #     # self.get_logger().info(f'Finished first write to csv file...')
        #     # with open(directory_csv, 'a') as f_object4:
            
        #     #     # Pass this file object to csv.writer()
        #     #     # and get a writer object
        #     #     writer_object = writer(f_object4)
            
        #     #     # Pass the list as an argument into
        #     #     # the writerow()
        #     #     writer_object.writerow(csv_list_formation)
            
        #     #     # Close the file object
        #     #     f_object4.close()

        # else:
        #     # Open our existing CSV file in append mode
        #     # Create a file object for this file
        #     with open(directory_csv, 'a') as f_object4:
            
        #         # Pass this file object to csv.writer()
        #         # and get a writer object
        #         writer_object = writer(f_object4)
            
        #         # Pass the list as an argument into
        #         # the writerow()
        #         writer_object.writerow(csv_list_formation)
            
        #         # Close the file object
        #         f_object4.close()

        # List that we want to add as a new row for data collection
        csv_list_formation = [
        behavior_timestamp,
        self.drone_pos_if[0,0], self.drone_pos_if[1,0], self.drone_pos_if[2,0],
        swarm_goal_pos[0,0], swarm_goal_pos[1,0], swarm_goal_pos[2,0], 

        #self.drone_swarm_dictionary['position_'+f'{1}'], self.drone_swarm_dictionary['position_'+f'{2}'], self.drone_swarm_dictionary['position_'+f'{3}'], 
        avg_swarm_pos[0,0], avg_swarm_pos[1,0], avg_swarm_pos[2,0],

        self.swarm_formation['form_uav'+ f'{1}'], self.swarm_formation['form_uav'+ f'{2}'], self.swarm_formation['form_uav'+ f'{3}'],
        self.form_assign_node_id,

        self.goal_form_pos[0,0], self.goal_form_pos[1,0], self.goal_form_pos[2,0],
        form_pos_dist_xy,
        max_ratio_avoidance,

        self.f_weights[0,0], self.f_weights[0,1], self.f_weights[0,2],  f_weight_avoid[0,0], f_weight_avoid[0,1], f_weight_avoid[0,2], self.f_weights[0,3],
        self.unit_vector_array[0, :], self.unit_vector_array[1, :], self.unit_vector_array[2, :], unit_vector_avoid[:,0], unit_vector_avoid[:,1], unit_vector_avoid[:,2], #self.unit_vector_array[3, :], 
        
        vel_cmd_out[0,0], vel_cmd_out[1,0], vel_cmd_out[2,0], vect_avoidance_sum[0,0], vect_avoidance_sum[1,0], vect_avoidance_sum[2,0],
        vel_cmd_out_rescale[0,0], vel_cmd_out_rescale[1,0], vel_cmd_out_rescale[2,0],

        behavior_ctrl_out_accel[0,0], behavior_ctrl_out_accel[1,0], behavior_ctrl_out_accel[2,0],
        behavior_ctrl_out_pos[0,0], behavior_ctrl_out_pos[1,0], behavior_ctrl_out_pos[2,0],
        ]
        # self.get_logger().info(f"SWARM FORMATION 7 (passed to CSV)  {self.swarm_formation} ")
        csv_list_header_formation = [
        "behavior_timestamp",

        "GPS_Pos_IF_X", "GPS_Pos_IF_Y", "GPS_Pos_IF_Z",
        "Swarm_Goal_Pos_X", "Swarm_Goal_Pos_Y", "Swarm_Goal_Pos_Z", 

        #"UAV_1_Pos", "UAV_2_Pos", "UAV_3_Pos", 
        "avg_UAV_Pos_X", "avg_UAV_Pos_Y", "avg_UAV_Pos_Z", 

        "Form_Pos_UAV_1", "Form_Pos_UAV_2", "Form_Pos_UAV_3", 
        "UAV_Form_Node_ID",

        "Goal_UAV_Form_Pos_X", "Goal_UAV_Form_Pos_Y", "Goal_UAV_Form_Pos_Z",
        "form_pos_dist_xy",
        "max_ratio_avoidance",

        "f_weights altitude", "f_weights goal", "f_weights formation", 'drone_1_f_weights UAV avoidance', 'drone_2_f_weights UAV avoidance', 'drone_3_f_weights UAV avoidance', #"f_weights UAV avoidance",
        "unit_vector_array altitude", "unit_vector_array goal", "unit_vector_array formation", 'drone_1_unit_vector_array UAV avoidance','drone_2_unit_vector_array UAV avoidance', 'drone_3_unit_vector_array UAV avoidance', #"unit_vector_array UAV avoidance",

        "vel_cmd_out[x]", "vel_cmd_out[y]", "vel_cmd_out[z]", "vect_avoidance_sum_X",  "vect_avoidance_sum_Y", "vect_avoidance_sum_Z",
        "vel_cmd_out_rescale[x]", "vel_cmd_out_rescale[y]", "vel_cmd_out_rescale[z]",

        "behavior_ctrl_out_accel[0]", "behavior_ctrl_out_accel[1]", "behavior_ctrl_out_accel[2]",
        "behavior_ctrl_out_pos[x]", "behavior_ctrl_out_pos[y]", "behavior_ctrl_out_pos[z]",
        ]

        directory_csv = os.path.join('/home/cnchano/', self.drone_namespace + '_behavior_calc_formation.csv') #os.path.join(os.getcwd(), 'src/drone_control/behavior_ctrl_out.csv')
        # self.get_logger().info(f'Getting csv file path: {directory_csv}...')
        if (self.iter_count == 0.0): #self.prev_ctrl_loop_timestamp is initialized at zero in the initialization function, therefore the first run of the script should have this value equal to zero
            # self.get_logger().info(f'creating Behavior Ctrl csv file...')
            with open(directory_csv, 'w', newline='') as f_object4:
                writer_object = writer(f_object4)
                writer_object.writerow(csv_list_header_formation)
                writer_object.writerow(csv_list_formation)
                # Close the file object
                f_object4.close()
                self.iter_count += 1
            # self.get_logger().info(f'Finished first write to csv file...')
            # with open(directory_csv, 'a') as f_object4:
            
            #     # Pass this file object to csv.writer()
            #     # and get a writer object
            #     writer_object = writer(f_object4)
            
            #     # Pass the list as an argument into
            #     # the writerow()
            #     writer_object.writerow(csv_list_formation)
            
            #     # Close the file object
            #     f_object4.close()

        else:
            # Open our existing CSV file in append mode
            # Create a file object for this file
            with open(directory_csv, 'a') as f_object4:
            
                # Pass this file object to csv.writer()
                # and get a writer object
                writer_object = writer(f_object4)
            
                # Pass the list as an argument into
                # the writerow()
                writer_object.writerow(csv_list_formation)
            
                # Close the file object
                f_object4.close()

        # #////////////////////////////////////////////////////////////////////////////////
        # #////////////////////////////////////////////////////////////////////////////////
        return [vel_cmd_out_rescale, goal_psi_y_if, behavior_timestamp, behavior_ctrl_out_pos, behavior_ctrl_out_accel, goal_ang_vel_psi_if, vel_cmd_goal_yaw_if, swarm_goal_pos]



    # def vectors_smallest_angle_2d (self, vel_a, vel_b):
    #     # Assuming you have two column vectors as 2D NumPy arrays
    #     # For example:
    #     vector_a = vel_a[0:1,0]

    #     vector_b = vel_b[0:1,0]

    #     # Step 1: Calculate the dot product
    #     dot_product = np.dot(vector_a.T, vector_b)  # Transpose of vector_a to make it a row vector

    #     # Step 2: Calculate the magnitudes
    #     magnitude_a = np.linalg.norm(vector_a)
    #     magnitude_b = np.linalg.norm(vector_b)

    #     # Step 3: Calculate the angle in radians
    #     angle_radians = np.arccos(dot_product / (magnitude_a * magnitude_b))

    #     # Step 4: Convert the angle to degrees
    #     angle_degrees = np.degrees(angle_radians)

    #     print("Angle between the two vectors (in degrees):", angle_degrees[0][0])

    #     return angle_degrees
    def rotate_2d(self, vector, angle):
        #Take 3-d vector and compute 2-d rotation
        vector_2d = vector[0:2]
        R_matrix = np.array([[np.cos(angle),-np.sin(angle)],[np.sin(angle),np.cos(angle)]])
        vector_2d = np.matmul(R_matrix, vector_2d)
        vector_out = np.zeros((3,1))
        vector_out[0:2] = vector_2d
        return vector_out

    def collision_avoidance_vector_calc(self, vel_self, vel_other):

        vel_self_2d = np.array([[np.asscalar(vel_self[0])],[np.asscalar(vel_self[1])]])
 
        vel_other_2d = np.array([[vel_other[0]],[vel_other[1]]])
        # print("Vector 2d:")
        # print(vel_self_2d)
        # print(vel_other_2d)
        v_self_magnitude = (vel_self_2d[0]**2 + vel_self_2d[1]**2)**0.5
        v_other_magnitude = (vel_other_2d[0]**2 + vel_other_2d[1]**2)**0.5
        if (v_self_magnitude > 1*(10**(-4))): #or v_other_norm > 1*(10**(-1)) )
            v_self_norm = vel_self_2d/v_self_magnitude
        else:
            v_self_norm = np.zeros((2,1))
        if (v_other_magnitude > 1*(10**(-4))):
            v_other_norm = vel_other_2d/v_other_magnitude #((vel_other[0,0]**2 + vel_other[1,0]**2)**0.5) #np.linalg.norm(vel_other_2d)
        else:
            v_other_norm = np.zeros((2,1))

        # print("Vector norms:")
        # print(v_self_norm)
        # print(v_other_norm)
        #Calculate bisecting vector between the two velocity vectors
        vect_bisect =  v_self_norm + v_other_norm
        v_bisect_magnitude = (vect_bisect[0]**2 + vect_bisect[1]**2)**0.5
        vect_bisect = vect_bisect/v_bisect_magnitude #normalize vector
        print("vect bisect")
        print(vect_bisect)
        #Check direction of vect_bisect to make sure it faces toward the left of self
        v_dir_check = np.asscalar(np.cross(vect_bisect, v_self_norm, axis=0))
        if (v_dir_check > 0): #v_bisect is to the right of vel_self and we want it to be to the left
            v_avoid = self.rotate_2d(vect_bisect, np.pi)
            # print("v_bisect is to the right of v_self")
        elif (v_dir_check == 0): #v_bisect is 0deg or 180deg to vel_self
            v_avoid = np.zeros((3,1))
            v_avoid[0:2] = vect_bisect
            # print("v_bisect is colinear with v_self")
        else: #v_bisect is to the right of vel_self
            # print("v_bisect is to the left of v_self")
            v_avoid = np.zeros((3,1))
            v_avoid[0:2] = vect_bisect
        return v_avoid

    def interagent_time2collision(self, agent_radius, vel_self, pos_self, vel_other, pos_other):
        #reformat arrays
        vel_self_2d = np.array([[np.asscalar(vel_self[0])],[np.asscalar(vel_self[1])]])
        vel_other_2d = np.array([[vel_other[0]],[vel_other[1]]])
        pos_self_2d = np.array([[np.asscalar(pos_self[0])],[np.asscalar(pos_self[1])]])
        pos_other_2d = np.array([[pos_other[0]],[pos_other[1]]])
        #Calculate coefficients of parabolic polynomial
        a = (vel_self_2d[0,0] - vel_other_2d[0,0])**2 + (vel_self_2d[1,0] - vel_other_2d[1,0])**2 #coefficients of t^2 term
        b = 2*((pos_self_2d[0,0] - pos_other_2d[0,0])*(vel_self_2d[0,0] - vel_other_2d[0,0]) + (pos_self_2d[1,0] - pos_other_2d[1,0])*(vel_self_2d[1,0] - vel_other_2d[1,0])) #coefficients of t term
        c = (pos_self_2d[0,0] - pos_other_2d[0,0])**2 + (pos_self_2d[1,0] - pos_other_2d[1,0])**2 - (2*agent_radius)**2 #coefficients of constants term #subtract radii of each agent (the same in our case) to the power of two
        #Use the discriminant to check for collision before taking the square root
            #IF D > 0, then there are two collision points (entry and exit) (past or present)
            #IF D = 0, then there is one collision points (glancing collision) (past or present)
            #IF D < 0, then there are no collision points (drones never collide)
        #Also check to ensure that the dot product of pos_other and vel_other relative to drone_self is decreasing by a significant speed. The "b" term is equivalent to the dot product
            #IF b > 0, then the other drone is moving away from drone self
            #IF b < 0, then the other drone is moving towards drone self
            #IF b = 0, then the other drone is remaining at a constant distance from drone self
            
        discriminant = (b**2) - (4*a*c)
        # print("discriminant")
        # print(discriminant)
        # print("dot product")
        # print(b)
        if (discriminant > 0 and b < -1*(10**(-4))):
            discriminant_sqrt = (discriminant)**0.5
            time_1 = (- b - discriminant_sqrt)/(2*a)
            time_2 = (- b + discriminant_sqrt)/(2*a)
            
            if time_1 >= 0 and time_2 >= 0:
                time_collision = min(time_1, time_2)
            else:
                time_collision = max(time_1, time_2)
                
                #check if collision happened already #NOTE: MAY NOT BE NECESSARY SINCE TIME IS STILL NEEDED FOR WEIGHT VECTOR
                # if ( time_1 < 0 and time_2 > 0 and b <= -1e-6)
                    # print("already collided...")
                    # return
                # else:
                    # t = max(t1, t2)
            collision_bool = True
            # print("head on collision")
        elif (discriminant == 0 and b < -1*(10**(-4))): 
            time_collision = -b/(2*a) #find collision time of single root
            collision_bool = True
            # print("glancing collision")
        else:
            # print("will not collide")
            collision_bool = False
            time_collision = None
            return time_collision, collision_bool
            
        return time_collision, collision_bool
    
    def array_has_next_value(self, arr, current_index):
        try:
            next_value = arr[current_index + (2*3)]
            return False
        except IndexError:
            return True
    
    def dist_calc_2d(self, Vgoal, Vref):
        """
        Calculate distance along x-y plane, taking two vectors of at least size = 2
        """
        dist = ((Vgoal[0] - Vref[0])**2 + (Vgoal[1] - Vref[1])**2)**0.5

        #((swarm_goal_pos[0] - self.drone_pos_if[0])**2 + (swarm_goal_pos[1] - self.drone_pos_if[1])**2)**(0.5)
        return dist
    
    def dist_calc_3d(self, Vgoal, Vref):
        """
        Calculate distance taking two vectors of at least size = 3
        """
        dist = ((Vgoal[0] - Vref[0])**2 + (Vgoal[1] - Vref[1])**2 + (Vgoal[2] - Vref[2])**2)**0.5

        #((swarm_goal_pos[0] - self.drone_pos_if[0])**2 + (swarm_goal_pos[1] - self.drone_pos_if[1])**2)**(0.5)
        return dist
    
    def vector_angle_2d(self, Vb):
        #Uses the right hand rule about the z-axis. Considering the x-axis as the reference
        #NOTE: ALL VECTORS ARE ROW VECTORS
        Vref_x = np.array([1.0,0.0,0.0])
        #input two column vector arrays in 2-d that are normalized
        Vn = np.array([0.0,0.0,1.0]) #normal vector in z-axis
        angle = np.arctan2(np.dot(np.cross(Vref_x, Vb), Vn), np.dot(Vref_x, Vb)) #result in radians
        if (angle < 0):
            angle = 2*np.pi + angle
        #angle = angle*(180/np.pi)
        #print(angle)
        return angle

    def Rot_bf_to_if(self, Vb_rt, phi_rt, theta_rt, psi_rt):
        # Rotational matrix that relates body frame velocity to inertial frame velocity
        #Phi - roll; Theta - pitch; Psi - Yaw
        R_x = np.array([[1, 0, 0],[0, np.cos(phi_rt), -np.sin(phi_rt)],[0, np.sin(phi_rt), np.cos(phi_rt)]])
        R_y = np.array([[np.cos(theta_rt),0,np.sin(theta_rt)],[0,1,0],[-np.sin(theta_rt),0,np.cos(theta_rt)]])
        R_z = np.array([[np.cos(psi_rt),-np.sin(psi_rt),0],[np.sin(psi_rt),np.cos(psi_rt),0],[0,0,1]])

        R_matrix = np.matmul(R_z,np.matmul(R_y,R_x))

        Vi_rt = np.matmul(R_matrix, Vb_rt)
    
        return Vi_rt
    
 
    def euler_from_quaternion(self, x, y, z, w): 
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = np.arctan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = np.arcsin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = np.arctan2(t3, t4)
     
        euler_angles = np.array([[roll_x],[pitch_y],[yaw_z]])
        return euler_angles # in radians
    
    def rotate_formation_single_file(self,swarm_size, formation_spacing, swarm_max_angle, past_swarm_goal_pos, current_form_lead_vector, swarm_goal_pos):
        # #----------------------------------------------------------------
        # #get normalized vector that depicts the formation by creating a vector from the 1st and last drone positions (id=1 to id=swarm_size) at the current drone swarm center
        # form_lead_vector = past_swarm_formation['form_uav'+ f'{1}'] - avg_swarm_pos #translate only the 1st position/drone_id=1 of the drone formation
        # #print(form_lead_vector)
        # form_lead_vector = form_lead_vector/(np.sqrt((form_lead_vector[0]**2)+(form_lead_vector[1]**2)+(form_lead_vector[2]**2)))
        # form_lead_vector_row =  np.array([form_lead_vector[0,0], form_lead_vector[1,0], form_lead_vector[2,0]])
        # #print(form_lead_vector)
        # #print(form_lead_vector_row)
        # #----------------------------------------------------------------
        # #get normalized vector to the goal point from the drone swarm center
        # swarm_goal_loc_row =  np.array([swarm_goal_pos[0,0] - avg_swarm_pos[0,0], swarm_goal_pos[1,0] - avg_swarm_pos[1,0], 0.0 - avg_swarm_pos[2,0]])
        # #print(swarm_goal_loc_row)
        # swarm_goal_loc_row = swarm_goal_loc_row/(np.sqrt((swarm_goal_loc_row[0]**2)+(swarm_goal_loc_row[1]**2)+(swarm_goal_loc_row[2]**2)))
        # #print(swarm_goal_loc_row)

        # #get the angle between the line formation and the goal point
        # form2goal_angle = self.vector_angle_2d_swarm_form(form_lead_vector_row, swarm_goal_loc_row)
        # #print(form2goal_angle*(180/np.pi))
        # #----------------------------------------------------------------
        # #Determine if the formation has completely rotated to the new goal direction
        # total_angle_increments = form2goal_angle/swarm_max_angle

        # if (total_angle_increments >= 1):
        #     cmd_rotation_angle = np.copysign(swarm_max_angle, form2goal_angle)
        #     rotation_complete_bool = False
        # else:
        #     cmd_rotation_angle = form2goal_angle
        #     rotation_complete_bool = True
        # #----------------------------------------------------------------
        # self.get_logger().info(f"SWARM FORMATION 1 (before rotation)  {past_swarm_formation} ")
        # swarm_formation = {}
        # past_swarm_formation_copy = past_swarm_formation.copy()
        # #Find the new points of the rotated formation about swarm origin, then convert to inertial frame coordinates
        # swarm_formation = self.rotate_formation_z(swarm_size, past_swarm_formation_copy, cmd_rotation_angle, avg_swarm_pos)
        # self.get_logger().info(f"SWARM FORMATION 2 (after rotation)  {swarm_formation} ")

        # return swarm_formation, rotation_complete_bool
    
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
        # self.get_logger().info(f"SWARM FORMATION 1 (initial)  {swarm_formation} ")
        #----------------------------------------------------------------

        #----------------------------------------------------------------
        #find distance to the formation center from inertial frame origin
        form_center_pos = self.formation_center_calc(swarm_size, swarm_formation)
        #----------------------------------------------------------------
        #Translate formation center location to the inertial frame origin
        swarm_formation = self.translate_formation(swarm_size, swarm_formation, np.absolute(form_center_pos))
        # self.get_logger().info(f"SWARM FORMATION 2 (translate to origin)  {swarm_formation} ")
        #----------------------------------------------------------------
        #Translate formation center to the drone swarm center
        swarm_formation = self.translate_formation(swarm_size, swarm_formation, past_swarm_goal_pos)
        # self.get_logger().info(f"SWARM FORMATION 3 (translate to avg_swarm_pos)  {swarm_formation} ")
        #----------------------------------------------------------------
        #get normalized vector that depicts the formation by creating a vector from the 1st and last drone positions (id=1 to id=swarm_size) at the current drone swarm center
        form_lead_vector = swarm_formation['form_uav'+ f'{1}'] - past_swarm_goal_pos#swarm_formation['form_uav'+ f'{swarm_size}']#translate only the 1st position/drone_id=1 of the drone formation
        #print(form_lead_vector)
        form_lead_vector = form_lead_vector/(np.sqrt((form_lead_vector[0]**2)+(form_lead_vector[1]**2)+(form_lead_vector[2]**2)))
        form_lead_vector_row =  np.array([form_lead_vector[0,0], form_lead_vector[1,0], form_lead_vector[2,0]])
        #print(form_lead_vector)
        #print(form_lead_vector_row)
        #----------------------------------------------------------------
        #get normalized vector to the current point of the lead drone from the drone swarm center
        current_form_lead_vector = current_form_lead_vector/(np.sqrt((current_form_lead_vector[0]**2)+(current_form_lead_vector[1]**2)+(current_form_lead_vector[2]**2)))
        current_form_lead_vector_row =  np.array([current_form_lead_vector[0,0], current_form_lead_vector[1,0], current_form_lead_vector[2,0]])
        #print(swarm_goal_loc_row)

        #get normalized vector to the goal point from the drone swarm center
        swarm_goal_loc_row =  np.array([swarm_goal_pos[0,0] - past_swarm_goal_pos[0,0], swarm_goal_pos[1,0] - past_swarm_goal_pos[1,0], 0.0 - past_swarm_goal_pos[2,0]])
        #print(swarm_goal_loc_row)
        swarm_goal_loc_row = swarm_goal_loc_row/(np.sqrt((swarm_goal_loc_row[0]**2)+(swarm_goal_loc_row[1]**2)+(swarm_goal_loc_row[2]**2)))
        #print(swarm_goal_loc_row)
        #----------------------------------------------------------------

        #get the angle between the line formation and the goal point
        form2current_form_angle = self.vector_angle_2d_swarm_form(form_lead_vector_row, current_form_lead_vector_row)
        #print(form2goal_angle*(180/np.pi))
        current_form2goal_angle = self.vector_angle_2d_swarm_form(form_lead_vector_row, current_form_lead_vector_row)
        
        #Determine if the formation has completely rotated to the new goal direction
        total_angle_increments = current_form2goal_angle/swarm_max_angle

        if (total_angle_increments >= 1):
            cmd_rotation_angle =  form2current_form_angle + np.copysign(swarm_max_angle, current_form2goal_angle)
            rotation_complete_bool = False
        else:
            cmd_rotation_angle = form2current_form_angle + current_form2goal_angle
            rotation_complete_bool = True

        #----------------------------------------------------------------
        #Find the new points of the rotated formation about swarm origin, then convert to inertial frame coordinates
        swarm_formation = self.rotate_formation_z(swarm_size, swarm_formation, cmd_rotation_angle, past_swarm_goal_pos)
        # self.get_logger().info(f"SWARM FORMATION 4 (rotate the formation towards the goal)  {swarm_formation} ")
        
        return swarm_formation, rotation_complete_bool
    #===========================================================================

    #///////////////////////////////////////////////////////////////////////////
    #FORMATION CALC FUNCTIONS
    
    #===========================================================================
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

        Vi_rt = np.dot(R_z, Vb_rt)

        return Vi_rt
    
    def swarm_center_calc(self, form_node_size, ignore_drone_id_array, swarm_size, drone_swarm_dictionary): #Find center of drone swarm from current positions
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
        new_swarm_form = {}
        for i in range(swarm_size): #create a dictionary to hold the drone data of the number of drones specified
            form_id_iter = int(i+1)
            
            # swarm_formation_pos_vect = swarm_formation['form_uav'+ f'{form_id_iter}'] - avg_swarm_pos #convert to swarm origin
            # # print(swarm_formation_pos_vect)
            # swarm_formation['form_uav'+ f'{form_id_iter}'] = self.Rot_z(swarm_formation_pos_vect,form2goal_angle) #rotate about swarm origin
            # swarm_formation['form_uav'+ f'{form_id_iter}'] = swarm_formation['form_uav'+ f'{form_id_iter}'] + avg_swarm_pos #convert back to inertial frame
            # #print(swarm_formation)

            swarm_formation_pos_vect = swarm_formation['form_uav'+ f'{form_id_iter}'] - avg_swarm_pos
            # self.get_logger().info(f"SWARM FORMATION 3 (after translation to swarm center)  {swarm_formation_pos_vect} ")
            rotated_swarm_formation_pos_vect = self.Rot_z(swarm_formation_pos_vect, form2goal_angle)
            # self.get_logger().info(f"SWARM FORMATION 4 (after rotation about z)  {rotated_swarm_formation_pos_vect} ")
            updated_swarm_formation_pos_vect = rotated_swarm_formation_pos_vect + avg_swarm_pos
            # self.get_logger().info(f"SWARM FORMATION 5 (after translation to world origin)  {updated_swarm_formation_pos_vect} ")
            new_swarm_form['form_uav'+ f'{form_id_iter}'] = updated_swarm_formation_pos_vect
        # self.get_logger().info(f"SWARM FORMATION 6 (after passing formation to dictionary)  {new_swarm_form} ")
        return new_swarm_form
    


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
        # self.get_logger().info(f"SWARM FORMATION 1 (initial)  {swarm_formation} ")
        #----------------------------------------------------------------

        #----------------------------------------------------------------
        #find distance to the formation center from inertial frame origin
        form_center_pos = self.formation_center_calc(swarm_size, swarm_formation)
        #----------------------------------------------------------------
        #Translate formation center location to the inertial frame origin
        swarm_formation = self.translate_formation(swarm_size, swarm_formation, np.absolute(form_center_pos))
        # self.get_logger().info(f"SWARM FORMATION 2 (translate to origin)  {swarm_formation} ")
        #----------------------------------------------------------------
        #Translate formation center to the drone swarm center
        swarm_formation = self.translate_formation(swarm_size, swarm_formation, avg_swarm_pos)
        # self.get_logger().info(f"SWARM FORMATION 3 (translate to avg_swarm_pos)  {swarm_formation} ")
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
        # self.get_logger().info(f"SWARM FORMATION 4 (rotate the formation towards the goal)  {swarm_formation} ")
        
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
        form_lead_vector_row =  np.array([form_lead_vector[1,0], form_lead_vector[0,0], form_lead_vector[2,0]])
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
    #===========================================================================

    #///////////////////////////////////////////////////////////////////////////
    
    #===========================================================================


    def drone_com_in_service(self):
        
        self.drone_com_in_future = self.drone_com_in_client.call_async(self.drone_com_in_req) 
        rclpy.spin_until_future_complete(self.drone_com_in_sub_node, self.drone_com_in_future)
        
        return self.drone_com_in_future.result()
    
    def call_imu_service(self):
        
        self.imu_req.drone_name = 'drone1'

        self.imu_future = self.imu_client.call_async(self.imu_req) 
        rclpy.spin_until_future_complete(self.imu_sub_node, self.imu_future)
        
        return self.imu_future.result()

    def call_gps_service(self):

        self.gps_req.drone_name = 'drone1'

        self.gps_future = self.gps_client.call_async(self.gps_req)
        rclpy.spin_until_future_complete(self.gps_sub_node, self.gps_future)
        return self.gps_future.result()

    def call_sim_clock_service(self):

        self.sim_clock_future = self.sim_clock_client.call_async(self.sim_clock_req)
        rclpy.spin_until_future_complete(self.sim_clock_sub_node, self.sim_clock_future)
        return self.sim_clock_future.result()

    def call_vel_calc_service(self):

        self.vel_calc_future = self.vel_calc_client.call_async(self.vel_calc_req)
        rclpy.spin_until_future_complete(self.vel_calc_sub_node, self.vel_calc_future)
        return self.vel_calc_future.result()


def main(args=None):
    rclpy.init(args=args)

    behavior_controller = BehaviorController()

    rclpy.spin(behavior_controller)

    #/////////////////////////
    #MULTI-THREADING CALLBACKS OPTION
    # executor = MultiThreadedExecutor() #add multi-threading to the node
    # executor.add_node(behavior_controller)
    # try:
    #     behavior_controller.get_logger().info('Beginning client, shut down with CTRL-C')
    #     executor.spin()
    # except KeyboardInterrupt:
    #     behavior_controller.get_logger().info('Keyboard interrupt, shutting down.\n')
    #///////////////////////////

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    behavior_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()