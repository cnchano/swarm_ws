import rclpy
from rclpy.node import Node

from drone_common.srv import BehaviorControl
from drone_common.srv import PosControl
from drone_common.srv import IMU
from drone_common.srv import GPStoENU
from drone_common.srv import VelCalc
from drone_common.srv import SimClock

import numpy as np
from csv import writer
import os
from .constants import (
    POS_CONTROL,
    DRONE_CONSTANT,
    POLES,
    BEHAVIORAL_CTRL,
    FILTER_CONSTANT,
)
class PositionController(Node):

    def __init__(self):
        super().__init__('position_controller')
        
        #Service to provide position controller outputs
        self.srv = self.create_service(
            PosControl, 
            'pos_ctrl_service', 
            self.pos_controller_service_callback
        )

        #//////////////////////////////////
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
        
        #Initialize Behavioral Control Client
        self.behavior_ctrl_sub_node = rclpy.create_node('behavior_ctrl_sub_node') #create sub-node for behavior control client
        self.behavior_ctrl_client = self.behavior_ctrl_sub_node.create_client(BehaviorControl, 'behavior_ctrl_service')
        while not self.behavior_ctrl_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Behavior service not available, waiting again...')
        self.behavior_ctrl_req = BehaviorControl.Request()

        self.vel_calc_sub_node = rclpy.create_node('vel_sub_node') #create sub-node for vel calc client
        #Initialize Vel Calc Client
        self.vel_calc_client = self.vel_calc_sub_node.create_client(VelCalc, 'vel_calc_service')
        while not self.vel_calc_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Vel Calc service not available, waiting again...')
        self.vel_calc_req = VelCalc.Request()

        self.sim_clock_sub_node = rclpy.create_node('sim_clock_sub_node') #create sub-node for sim clock client
        #Initialize sim_clock Client
        self.sim_clock_client = self.sim_clock_sub_node.create_client(SimClock, 'sim_clock_service')
        while not self.sim_clock_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('SIM CLOCK service not available, waiting again...')
        self.sim_clock_req = SimClock.Request()
        
        #//////////////////////////////////
        #ROS2 Parameter
        #Get drone namespace to extract its own drone_id
        self.declare_parameter('drone_ns')
        self.drone_namespace = self.get_parameter('drone_ns').get_parameter_value().string_value 
        #//////////////////////////////////
        #Constants
        self.GRAVITY = DRONE_CONSTANT['g']
        self.MASS = DRONE_CONSTANT['mass']
        self.Ixx =  DRONE_CONSTANT['Ixx']
        self.Iyy =  DRONE_CONSTANT['Iyy']
        self.Izz =  DRONE_CONSTANT['Izz']

        #Initialize variables
        self.drone_lin_vel_bf = np.zeros([3, 1])
        self.drone_lin_vel_if = np.zeros([3, 1])
        self.drone_orient = np.zeros([4, 1])
        self.drone_pos_if = np.zeros([3, 1])
        self.drone_lin_accel_bf = np.zeros([3, 1])
        self.drone_lin_accel_if = np.zeros([3, 1])

        #Pos PID integral
        self.PID_INTEGRAL_WINDOW_SIZE = FILTER_CONSTANT['pid_integral_window']
        self.pos_err_integral_array = np.zeros([3, 1])
        self.yaw_err_integral_array = np.zeros([1, 1])

        #Pos PID moving average
        self.PID_ACCEL_AVG_WINDOW_SIZE = FILTER_CONSTANT['pid_accel_avg_window']
        self.pid_accel_matrix = np.zeros([3,1])

        # #NOTE:TEMPORARY: X-Y goal location
        # self.GOAL_LOC_XY = BEHAVIORAL_CTRL['goal_loc_xy']
        #Goal behavior
        self.W_GOAL = BEHAVIORAL_CTRL['w_goal']
        self.BOUND_GOAL = BEHAVIORAL_CTRL['bound_goal']
        self.MIN_BOUND_GOAL = BEHAVIORAL_CTRL['min_bound_goal']

        # #NOTE:TEMPORARY: Altitude goal
        # self.GOAL_ALT = BEHAVIORAL_CTRL['goal_alt']

        self.BOUND_ALT_POS_CTRL = BEHAVIORAL_CTRL['bound_alt_pos_ctrl']

        self.goal_lin_vel_if = np.zeros([3, 1])
        self.goal_pos_if = np.zeros([3, 1])
        self.goal_lin_accel_if = np.zeros([3, 1])

        self.past_timestamp = 0.0
        self.past_pos_err = np.zeros([3, 1])
        self.past_yaw_err = np.zeros([1, 1])

        #NOTE: HEADING VARIABLES

        self.BOUND_YAW_POS_CTRL = BEHAVIORAL_CTRL['bound_yaw_pos_ctrl']

        #Initialize iteration counter for csv write to initialize file, then later add variables
        self.iter_count = 0.0

    def pos_controller_service_callback(self, request, response):
        
        
        #calculate new position data
        pos_control_data = self.pos_controller_calc()
        control_out_U1 = pos_control_data[0]
        control_out_U4 = pos_control_data[8]
        v1 = pos_control_data[9]
        PID_accel = pos_control_data[10]
        # goal_phi_r_if = pos_control_data[1]
        # goal_theta_p_if = pos_control_data[2]
        # goal_psi_y_if = pos_control_data[3]


        #pass data to msg
        #ensure numbers are float to prevent errors
        response.pos_control_out_u1 = float(control_out_U1)
        response.control_out_u4 = float(control_out_U4)
        response.v1 = float(v1)
        response.pid_accel = [float(PID_accel[0]), float(PID_accel[1]), float(PID_accel[2])]
        # response.pos_control_out_phi_ref = float(goal_phi_r_if)
        # response.pos_control_out_theta_ref = float(goal_theta_p_if)
        # response.pos_control_out_psi_ref = float(goal_psi_y_if)



        return response
        
    def pos_controller_calc(self):
        #import position control data as client/ convert IMU data WRT inertial frame
        behavioral_ctrl_data = self.call_behavior_ctrl_service()
        #self.get_logger().info(f'Behavior control service data: {behavioral_ctrl_data}')

        #import IMU data as client/ convert IMU data WRT inertial frame
        imu_data = self.call_imu_service()
        #self.get_logger().info(f'IMU service data: {imu_data}')

        #import GPS data as client/ convert IMU data WRT inertial frame
        gps_data = self.call_gps_service()
        #self.get_logger().info(f'GPS service data: {gps_data}')

        #import velocity
        velocity_data = self.call_vel_calc_service()

        #import timestamp
        clock_data = self.call_sim_clock_service()
        #self.get_logger().info(f'GPS service data: {clock_data}')

        #Pass timestamp data
        timestamp = clock_data.sim_timestamp


        #Pass behavior control data
        self.goal_lin_vel_if[0] = np.asarray(behavioral_ctrl_data.behavior_ctrl_out_vel_x)
        self.goal_lin_vel_if[1] = np.asarray(behavioral_ctrl_data.behavior_ctrl_out_vel_y)
        self.goal_lin_vel_if[2] = np.asarray(behavioral_ctrl_data.behavior_ctrl_out_vel_z)
        
        self.goal_pos_if[0] = behavioral_ctrl_data.behavior_ctrl_out_pos_x
        self.goal_pos_if[1] = behavioral_ctrl_data.behavior_ctrl_out_pos_y
        self.goal_pos_if[2] = behavioral_ctrl_data.behavior_ctrl_out_pos_z

        self.goal_lin_accel_if[0] = behavioral_ctrl_data.behavior_ctrl_out_accel_x
        self.goal_lin_accel_if[1] = behavioral_ctrl_data.behavior_ctrl_out_accel_y
        self.goal_lin_accel_if[2] = behavioral_ctrl_data.behavior_ctrl_out_accel_z

        goal_psi_y_if = behavioral_ctrl_data.goal_psi_y_if
        goal_ang_vel_psi_if = behavioral_ctrl_data.goal_ang_vel_psi_if
        vel_cmd_goal_yaw_if = behavioral_ctrl_data.vel_cmd_goal_yaw_if

        swarm_goal_pos = behavioral_ctrl_data.swarm_goal_pos



        self.drone_lin_accel_bf[0] = imu_data.drone_lin_accel_x
        self.drone_lin_accel_bf[1] = imu_data.drone_lin_accel_y
        self.drone_lin_accel_bf[2] = imu_data.drone_lin_accel_z

        self.drone_orient[0] = imu_data.drone_orient_x
        self.drone_orient[1] = imu_data.drone_orient_y
        self.drone_orient[2] = imu_data.drone_orient_z
        self.drone_orient[3] = imu_data.drone_orient_w

        drone_ang_vel_bf = np.zeros([3, 1])
        drone_ang_vel_bf[0] = imu_data.drone_ang_vel_x
        drone_ang_vel_bf[1] = imu_data.drone_ang_vel_y
        drone_ang_vel_bf[2] = imu_data.drone_ang_vel_z

        #Pass GPS data
        self.drone_pos_if[0] = gps_data.de #x-axis
        self.drone_pos_if[1] = gps_data.dn #y-axis
        self.drone_pos_if[2] = gps_data.du #z-axis

        #Pass Velocity data
        self.drone_lin_vel_if = np.array([[velocity_data.drone_vel_if[0]], [velocity_data.drone_vel_if[1]], [velocity_data.drone_vel_if[2]]])

      

        #convert quarternion orientation to euler angles
        self.drone_orient_euler = self.euler_from_quaternion(self.drone_orient[0], self.drone_orient[1], self.drone_orient[2], self.drone_orient[3])

        #Convert IMU angular rates from the body frame to the inertial frame
        drone_ang_vel_if = self.Transfer_bf_to_if(drone_ang_vel_bf, np.asscalar(self.drone_orient_euler[0]), np.asscalar(self.drone_orient_euler[1]))

        #Convert local velocity from body frame to inertial frame
        # self.drone_lin_vel_if = self.Rot_bf_to_if(self.drone_lin_vel_bf, self.drone_orient_euler[0], self.drone_orient_euler[1], self.drone_orient_euler[2]) #inputs: local velocity, roll, pitch, yaw
        self.drone_lin_accel_if = self.Rot_bf_to_if(self.drone_lin_accel_bf, self.drone_orient_euler[0], self.drone_orient_euler[1], self.drone_orient_euler[2])
        self.drone_lin_accel_if[2] = self.drone_lin_accel_if[2] - self.GRAVITY #remove gravity from IMU accel data
        

        #////////////////////////////////////////////////////////////////
        #/////////////////////////////////////////////////////////////// NOTE: Calculation for PID Altitude control
        #Input drone velocity error and output drone commanded acceleration for U1
        #Calculate position error
        pos_err = np.zeros([3,1])
        # pos_err[0:1] = self.goal_pos_if[0:1] - self.drone_pos_if[0:1]
        # pos_err[2] = self.goal_pos_if[2] - self.drone_pos_if[2] 
        pos_err = self.goal_pos_if - self.drone_pos_if

        if (np.absolute(swarm_goal_pos[2] - self.goal_pos_if[2]) <= self.BOUND_ALT_POS_CTRL): 
            Kp = POLES['ss_pos_alt_Kp']
            Kd = POLES['ss_pos_alt_Kd']
        else:
            Kp = POLES['pos_alt_Kp']
            Kd = POLES['pos_alt_Kd']
        
        #Derivative of goal altitude and current drone position. 
        pos_err_derivative = self.goal_lin_vel_if - self.drone_lin_vel_if
        pos_err_integral_dt = np.zeros([3,1])
        
        xy_dist = self.dist_calc_2d(swarm_goal_pos, self.goal_pos_if) #use to check if within goal point boundary distance for integral wind-up prevention
    
        #NOTE: INTEGRAL CALCULATION FOR PID #NOTE SOMETHING WRONG WITH INTEGRAL CALC FOR X-Y MOTION!!!
        if (np.absolute(swarm_goal_pos[2] - self.goal_pos_if[2]) <= self.BOUND_ALT_POS_CTRL): #Prevent integral wind-up while drone is constantly adjusting altitude
            #Calculate incremental integral between the last iteration and this current one
            if (xy_dist <= self.BOUND_GOAL): #prevent integral wind-up while reaching goal point
                pos_err_integral_dt = (timestamp - self.past_timestamp)*((self.past_pos_err + pos_err)*0.5)
            else:
                pos_err_integral_dt[2] = (timestamp - self.past_timestamp)*((self.past_pos_err[2] + pos_err[2])*0.5)
                pos_err_integral_dt[0] = 0.0
                pos_err_integral_dt[1] = 0.0
        else:
            pos_err_integral_dt = np.zeros([3,1])

        (arr_col_size, arr_row_size) = self.pos_err_integral_array.shape
        if (arr_row_size < self.PID_INTEGRAL_WINDOW_SIZE):
            self.pos_err_integral_array = np.append(self.pos_err_integral_array, pos_err_integral_dt, axis=1)
            
        else:
            self.pos_err_integral_array = np.append(self.pos_err_integral_array, pos_err_integral_dt, axis=1)
            
            self.pos_err_integral_array = np.delete(self.pos_err_integral_array, 0, 1)
        
        pos_err_integral = np.sum(self.pos_err_integral_array, axis=1)
        
        #////////////////////////////////////////////
        
        
        #Calculate commanded drone accel in Z direction
        command_accel = np.zeros([3, 1])
        #command_accel[2] =  POLES['alt_Kp']*vel_err[2] #POLES['alt_Kp']*accel_err[2]
        command_accel[0] =  POLES['pos_xy_Kp']*pos_err[0] + POLES['pos_xy_Kd']*pos_err_derivative[0] + POLES['pos_xy_Ki']*pos_err_integral[0] 
        command_accel[1] =  POLES['pos_xy_Kp']*pos_err[1] + POLES['pos_xy_Kd']*pos_err_derivative[1] + POLES['pos_xy_Ki']*pos_err_integral[1]
        command_accel[2] =  Kp*pos_err[2] + Kd*pos_err_derivative[2] + POLES['ss_pos_alt_Ki']*pos_err_integral[2]
        v1 = command_accel[2]
        
        #/////////////////////////
        #NOTE: MOVING AVERAGE FOR PID ACCEL OUTPUT
        (arr_col_size, arr_row_size) = self.pid_accel_matrix.shape
        if (arr_row_size < self.PID_ACCEL_AVG_WINDOW_SIZE):
            self.pid_accel_matrix = np.append(self.pid_accel_matrix, np.array([[command_accel[0,0]],[command_accel[1,0]], [command_accel[2,0]]]), axis=1)
            
        else:
            self.pid_accel_matrix = np.append(self.pid_accel_matrix, np.array([[command_accel[0,0]],[command_accel[1,0]], [command_accel[2,0]]]), axis=1)
            
            self.pid_accel_matrix = np.delete(self.pid_accel_matrix, 0, 1)
            
        kernel_array = np.ones(self.PID_ACCEL_AVG_WINDOW_SIZE)/self.PID_ACCEL_AVG_WINDOW_SIZE
        pid_arr_conv_roll = np.convolve(self.pid_accel_matrix[0,:], kernel_array, mode='valid')
        pid_arr_conv_pitch = np.convolve(self.pid_accel_matrix[1,:], kernel_array, mode='valid')
        pid_arr_conv_alt = np.convolve(self.pid_accel_matrix[2,:], kernel_array, mode='valid')
        
        avg_pid_accel = np.array([[pid_arr_conv_roll[-1]],[pid_arr_conv_pitch[-1]], [pid_arr_conv_alt[-1]]])


        #/////////////////////////

        #calculate U1 controller output
        accel_calc = command_accel[2] #+ self.GRAVITY
        denominator = (np.cos(np.asscalar(self.drone_orient_euler[0]))*np.cos(np.asscalar(self.drone_orient_euler[1])))
        control_out_U1 = (self.MASS*(accel_calc + self.GRAVITY))/denominator
        
        # self.get_logger().info(f'timestamp : {timestamp}')
        # self.get_logger().info(f'MASS : {self.MASS}')
        # self.get_logger().info(f'GRAVITY: {self.GRAVITY}')
        # self.get_logger().info(f'denominator data: {denominator}')
        # self.get_logger().info(f'control_out_U1 data: {control_out_U1}')
        
        #Set U1 to equal zero when the force required is negative. The propeller thrust is unidirectional.
        if (control_out_U1 <= 0):
            control_out_U1 = 0.0001

        #//////////////////////////////////////////////////////////////
        #////////////////////////////////////////////////////////////// 
        yaw_Kp = POLES['yaw_Kp']
        yaw_Ki = POLES['yaw_Ki']
        yaw_Kd = POLES['yaw_Kd']

        #Calculate yaw angle error
        yaw_err = goal_psi_y_if - self.drone_orient_euler[2]

        #Derivative of goal yaw position and current drone yaw position. 
        yaw_err_derivative = goal_ang_vel_psi_if - drone_ang_vel_if[2]
        
        #NOTE: INTEGRAL CALCULATION FOR PID
        if (np.absolute(vel_cmd_goal_yaw_if - self.drone_orient_euler[2]) <= self.BOUND_YAW_POS_CTRL): #Prevent integral wind-up while drone is constantly adjusting altitude
            #Calculate incremental integral between the last iteration and this current one
            yaw_err_integral_dt = np.array([[(timestamp - self.past_timestamp)*(np.asscalar(self.past_yaw_err + yaw_err)*0.5)]])
        else:
            yaw_err_integral_dt = np.array([[0.0]])

        (arr_col_size, arr_row_size) = self.yaw_err_integral_array.shape
        if (arr_row_size < self.PID_INTEGRAL_WINDOW_SIZE):
            # self.get_logger().info(f'IF self.yaw_err_integral_array: {self.yaw_err_integral_array}')
            # self.get_logger().info(f'IF yaw_err_integral_dt: {yaw_err_integral_dt}')
            self.yaw_err_integral_array = np.concatenate((self.yaw_err_integral_array, yaw_err_integral_dt), axis=1)
            
        else:
            # self.get_logger().info(f'ELSE self.yaw_err_integral_array: {self.yaw_err_integral_array}')
            # self.get_logger().info(f'ELSE yaw_err_integral_dt: {yaw_err_integral_dt}')

            self.yaw_err_integral_array = np.concatenate((self.yaw_err_integral_array, yaw_err_integral_dt), axis=1)
            
            self.yaw_err_integral_array = np.delete(self.yaw_err_integral_array, 0, 1)
        
        yaw_err_integral = np.sum(self.yaw_err_integral_array, axis=1)    

        if (np.absolute(swarm_goal_pos[2] - self.goal_pos_if[2]) <= self.BOUND_ALT_POS_CTRL): #Set heading controller to start once altitude has been reached.
 
            v4 =  yaw_Kp*yaw_err #+ yaw_Kd*yaw_err_derivative #+ yaw_Ki*yaw_err_integral
            
            #Feedback Linearization for YAW 
            I1 = (self.Ixx - self.Iyy)/self.Izz
            I2 = 1/self.Izz

            control_out_U4 = (-I1*drone_ang_vel_bf[0]*drone_ang_vel_bf[1] + v4)/I2
        else:
            #control_out_U4 = 0.0 
            #////////////// NOTE: TEMPORARILY HERE
            v4 =  yaw_Kp*yaw_err #+ yaw_Kd*yaw_err_derivative #+ yaw_Ki*yaw_err_integral
            
            #Feedback Linearization for YAW 
            I1 = (self.Ixx - self.Iyy)/self.Izz
            I2 = 1/self.Izz

            control_out_U4 = (-I1*drone_ang_vel_bf[0]*drone_ang_vel_bf[1] + v4)/I2
            #/////////////

        

        #//////////////////////////////////////////////////////////////
        #////////////////////////////////////////////////////////////// 
        # List that we want to add as a new row for data collection
        # csv_list = [
        # timestamp,
        # self.goal_pos_if[0,0], self.goal_pos_if[1,0], self.goal_pos_if[2,0], 
        # np.asscalar(self.goal_lin_vel_if[0,0]), np.asscalar(self.goal_lin_vel_if[1,0]), np.asscalar(self.goal_lin_vel_if[2,0]), 
        # self.goal_lin_accel_if[2,0], \
        # np.asscalar(self.drone_pos_if[0,0]), np.asscalar(self.drone_pos_if[1,0]), np.asscalar(self.drone_pos_if[2,0]), \
        # np.asscalar(self.drone_lin_vel_if[0,0]), np.asscalar(self.drone_lin_vel_if[1,0]), np.asscalar(self.drone_lin_vel_if[2,0]), 
        # drone_ang_vel_bf[0], drone_ang_vel_bf[1], drone_ang_vel_bf[2],
        # self.drone_orient_euler[0], self.drone_orient_euler[1], self.drone_orient_euler[2],
        # # K_CONST[0], K_CONST[1],
        # # err_x_if[0,0], err_x_if[1,0],
        # # err_y_if[0,0], err_y_if[1,0],
        # # err_z_if[0,0], err_z_if[1,0],
        # # accel_err_if[0,0], accel_err_if[1,0], accel_err_if[2,0], \
        # yaw_err,
        # yaw_err_integral,
        # yaw_err_derivative,
        # v4,

        # pos_err[0], pos_err[1], pos_err[2],
        # pos_err_integral[0], pos_err_integral[1], pos_err_integral[2],
        # pos_err_derivative[0], pos_err_derivative[1], pos_err_derivative[2],
        
        # avg_pid_accel[0], avg_pid_accel[1], avg_pid_accel[2],
        # command_accel[0], command_accel[1], command_accel[2],
        # self.goal_lin_accel_if[0,0], self.goal_lin_accel_if[1,0], self.goal_lin_accel_if[2,0], \
        # self.drone_lin_accel_if[0,0], self.drone_lin_accel_if[1,0], self.drone_lin_accel_if[2,0], \
        # #goal_phi_r_if, goal_theta_p_if, goal_psi_y_if, 
        # control_out_U1, 
        # control_out_U4,
        
        # #self.drone_lin_vel_bf[0], self.drone_lin_vel_bf[1], self.drone_lin_vel_bf[2], self.drone_lin_accel_bf[0], self.drone_lin_accel_bf[1], self.drone_lin_accel_bf[2], \
        # ]

        # csv_list_header = [
        # "timestamp",
        # "Goal_Pos_IF_X", "Goal_Pos_IF_Y", "Goal_Pos_IF_Z", 
        # "Goal_Vel_IF_X", "Goal_Vel_IF_Y", "Goal_Vel_IF_Z", 
        # "Goal_Accel_IF_Z", \
        # "GPS_Pos_IF_X", "GPS_Pos_IF_Y", "GPS_Pos_IF_Z", \
        # "Vel_IF_X", "Vel_IF_Y", "Vel_IF_Z", 
        # "drone_ang_vel_bf X", "drone_ang_vel_bf Y", "drone_ang_vel_bf Z",
        # "drone_orient_euler_roll", "drone_orient_euler_pitch", "drone_orient_euler_yaw",
        # # "K_CONST[0]", "K_CONST[1]",
        # # "err_x_if[0]", "err_x_if[1]",
        # # "err_y_if[0]", "err_y_if[1]",
        # # "err_z_if[0]", "err_z_if[1]",
        # # "accel_err_if[0]", "accel_err_if[1]", "accel_err_if[2]", \
        # "yaw_err",
        # "yaw_err_integral",
        # "yaw_err_derivative",
        # "V4",

        # "pos_err_X", "pos_err_Y", "pos_err_Z",
        # "pos_err_integral_X", "pos_err_integral_Y", "pos_err_integral_Z",
        # "pos_err_derivative_X", "pos_err_derivative_Y", "pos_err_derivative_Z",

        # "PID_avg_accel_X", "PID_avg_accel_Y", "PID_avg_accel_alt_Z",
        # "PID_accel_X", "PID_accel_Y", "PID_accel_alt_Z",
        # "Est_Accel_IF_X", "Est_Accel_IF_Y", "Est_Accel_IF_Z",\
        # "IMU_Accel_IF_X", "IMU_Accel_IF_Y", "IMU_Accel_IF_Z",\
        # #"goal_phi_r_if", "goal_theta_p_if", "goal_psi_y_if", 
        # "control_out_U1", 
        # "control_out_U4",
        
        # #"IMU_PC_Vel_BF_X", "IMU_PC_Vel_BF_Y", "IMU_PC_Vel_BF_Z", "IMU_PC_Accel_BF_X", "IMU_PC_Accel_BF_Y", "IMU_PC_Accel_BF_Z",
        # ]
        
        # directory_csv = os.path.join('/home/cnchano/', self.drone_namespace + '_pos_ctrl_out.csv') #os.path.join(os.getcwd(), 'src/drone_control/pos_ctrl_out.csv')
        # # self.get_logger().info(f'Getting csv file path: {directory_csv}...')
        # if (self.iter_count == 0.0): #self.prev_ctrl_loop_timestamp is initialized at zero in the initialization function, therefore the first run of the script should have this value equal to zero
        #     # self.get_logger().info(f'creating Pos Ctrl csv file...')
        #     with open(directory_csv, 'w', newline='') as f_object:
        #         writer_object = writer(f_object)
        #         writer_object.writerow(csv_list_header)
        #         writer_object.writerow(csv_list)
        #         # Close the file object
        #         f_object.close()
        #         self.iter_count += 1
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
        #///////////////////////////////////////////////////////////
        #/////////////////////////////////////////////////////////// NOTE: Save error to for next iteration when calculating PID integral
        #Save current values for the next calculation iteration
        self.past_timestamp = timestamp
        self.past_pos_err = pos_err
        self.past_yaw_err = yaw_err
        #///////////////////////////////////////////////////////////
        #///////////////////////////////////////////////////////////

        return [control_out_U1, #goal_phi_r_if, goal_theta_p_if, goal_psi_y_if,  
                self.goal_lin_vel_if, self.goal_pos_if, self.goal_lin_accel_if, 
                self.drone_lin_vel_if, self.drone_lin_accel_if, 
                self.goal_lin_accel_if, 
                self.drone_pos_if, control_out_U4, v1, avg_pid_accel ]

    def Rot_bf_to_if(self, Vb_rt, phi_rt, theta_rt, psi_rt):
        # Rotational matrix that relates body frame velocity to inertial frame velocity
        #Phi - roll; Theta - pitch; Psi - Yaw
        R_x = np.array([[1, 0, 0],[0, np.cos(phi_rt), -np.sin(phi_rt)],[0, np.sin(phi_rt), np.cos(phi_rt)]])
        R_y = np.array([[np.cos(theta_rt),0,np.sin(theta_rt)],[0,1,0],[-np.sin(theta_rt),0,np.cos(theta_rt)]])
        R_z = np.array([[np.cos(psi_rt),-np.sin(psi_rt),0],[np.sin(psi_rt),np.cos(psi_rt),0],[0,0,1]])

        R_matrix = np.matmul(R_z,np.matmul(R_y,R_x))

        Vi_rt = np.matmul(R_matrix, Vb_rt)
    
        return Vi_rt
    
    def Transfer_bf_to_if(self, Vb_rt, phi_rt, theta_rt):
        # Rotational matrix that relates body frame velocity to inertial frame velocity
        #Phi - roll; Theta - pitch; Psi - Yaw
        T_matrix = np.array([[1, np.sin(phi_rt)*np.tan(theta_rt), np.cos(phi_rt)*np.tan(theta_rt)],
                             [0, np.cos(phi_rt),                  -np.sin(phi_rt)],
                             [0, np.sin(phi_rt)/np.cos(theta_rt), np.cos(phi_rt)/np.cos(theta_rt)]])

        Vi_rt = np.matmul(T_matrix, Vb_rt)

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
    
    def dist_calc_2d(self, Vgoal, Vref):
        """
        Calculate distance along x-y plane, taking two vectors of at least size = 2
        """
        dist = np.sqrt((Vgoal[0] - Vref[0])**2 + (Vgoal[1] - Vref[1])**2)

        #((swarm_goal_pos[0] - self.drone_pos_if[0])**2 + (swarm_goal_pos[1] - self.drone_pos_if[1])**2)**(0.5)
        return dist

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

    def call_behavior_ctrl_service(self):

        self.behavior_ctrl_req.drone_name = 'drone1'

        self.behavior_ctrl_future = self.behavior_ctrl_client.call_async(self.behavior_ctrl_req)
        rclpy.spin_until_future_complete(self.behavior_ctrl_sub_node, self.behavior_ctrl_future)
        return self.behavior_ctrl_future.result()

    def call_vel_calc_service(self):

        self.vel_calc_future = self.vel_calc_client.call_async(self.vel_calc_req)
        rclpy.spin_until_future_complete(self.vel_calc_sub_node, self.vel_calc_future)
        return self.vel_calc_future.result()

    def call_sim_clock_service(self):

        self.sim_clock_future = self.sim_clock_client.call_async(self.sim_clock_req)
        rclpy.spin_until_future_complete(self.sim_clock_sub_node, self.sim_clock_future)
        return self.sim_clock_future.result()

def main(args=None):
    rclpy.init(args=args)

    position_controller = PositionController()

    rclpy.spin(position_controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    position_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()