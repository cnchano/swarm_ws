
import rclpy
from rclpy.node import Node

from drone_common.srv import BehaviorControl
from drone_common.srv import PosControl
from drone_common.srv import AttitudeControl
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
    ATTITUDE_CTRL,
    CTRL_DT,
)
class AttitudeController(Node):

    def __init__(self):
        super().__init__('attitude_controller')
        
        #Service to provide position controller outputs
        self.srv = self.create_service(
            AttitudeControl, 
            'attitude_ctrl_service', 
            self.attitude_controller_service_callback
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
        
        # Constants
        self.Ixx =  DRONE_CONSTANT['Ixx']
        self.Iyy =  DRONE_CONSTANT['Iyy']
        self.Izz =  DRONE_CONSTANT['Izz']
        self.J =  DRONE_CONSTANT['Jtp']
        #Initialize variables
        self.drone_lin_vel_bf = np.zeros([3, 1])
        self.drone_lin_vel_if = np.zeros([3, 1])
        self.drone_orient = np.zeros([4, 1])
        self.drone_pos_if = np.zeros([3, 1])
        self.drone_lin_accel_bf = np.zeros([3, 1])
        self.drone_lin_accel_if = np.zeros([3, 1])
        self.GRAVITY = DRONE_CONSTANT['g']
        self.MASS = DRONE_CONSTANT['mass']

        # self.GOAL_ALT = BEHAVIORAL_CTRL['goal_alt']
        self.BOUND_ALT_POS_CTRL = BEHAVIORAL_CTRL['bound_alt_pos_ctrl']

        self.BOUND_ANG_POS_RP = BEHAVIORAL_CTRL['bound_ang_roll&pitch']
        self.MAX_ANG_VEL_RP = BEHAVIORAL_CTRL['max_roll&pitch_ang_vel']
        self.W_RP = BEHAVIORAL_CTRL['w_roll&pitch'] 

        self.BOUND_ROLL_POS_CTRL = ATTITUDE_CTRL['bound_roll_pos_ctrl']
        self.BOUND_PITCH_POS_CTRL = ATTITUDE_CTRL['bound_pitch_pos_ctrl']

        self.PID_INTEGRAL_WINDOW_SIZE = FILTER_CONSTANT['pid_integral_window']

        self.phi_err_integral_array = np.zeros([1, 1])
        self.theta_err_integral_array = np.zeros([1, 1])

        self.goal_lin_vel_if = np.zeros([3, 1])
        self.goal_pos_if = np.zeros([3, 1])
        self.goal_lin_accel_if = np.zeros([3, 1])

        self.past_goal_orient_if = np.zeros([2,1])
        self.past_phi_err = 0.0 #roll
        self.past_theta_err = 0.0 #pitch

        self.past_timestamp = 0.0

        #Initialize iteration counter for csv write to initialize file, then later add variables
        self.iter_count = 0.0
        self.get_logger().info(f'Attitude controller initialized...')


    def attitude_controller_service_callback(self, request, response):
        
        U1 = request.control_out_u1
        past_omega_sum = request.past_omega_sum
        v1 = request.v1
        pid_accel = request.pid_accel
        #calculate new position data
        attitude_control_data = self.attitude_controller_calc(U1, past_omega_sum, v1, pid_accel)

        control_out_U2 = attitude_control_data[0]
        control_out_U3 = attitude_control_data[1]

        response.control_out_u2 = float(control_out_U2)
        response.control_out_u3 = float(control_out_U3)

        return response
        
    def attitude_controller_calc(self, U1, past_omega_sum, v1, pid_accel):
        #import position control data as client/ convert IMU data WRT inertial frame
        behavioral_ctrl_data = self.call_behavior_ctrl_service()
        # self.get_logger().info(f'Behavior control service data: {behavioral_ctrl_data}')

        #import IMU data as client/ convert IMU data WRT inertial frame
        imu_data = self.call_imu_service()
        # self.get_logger().info(f'IMU service data: {imu_data}')

        #import GPS data as client/ convert IMU data WRT inertial frame
        gps_data = self.call_gps_service()
        # self.get_logger().info(f'GPS service data: {gps_data}')

        #import velocity
        velocity_data = self.call_vel_calc_service()
        # self.get_logger().info(f'Velocity service data: {velocity_data}')

        #import timestamp
        clock_data = self.call_sim_clock_service()
        # self.get_logger().info(f'Clock service data: {clock_data}')

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

        #Pass IMU data
        #Convert velocity from body frame to inertial frame
        # self.drone_lin_vel_bf[0] = imu_data.drone_lin_vel_x
        # self.drone_lin_vel_bf[1] = imu_data.drone_lin_vel_y
        # self.drone_lin_vel_bf[2] = imu_data.drone_lin_vel_z

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
        self.drone_lin_accel_if = self.Rot_bf_to_if(self.drone_lin_accel_bf, self.drone_orient_euler[0], self.drone_orient_euler[1], self.drone_orient_euler[2])
        self.drone_lin_accel_if[2] = self.drone_lin_accel_if[2] - self.GRAVITY #remove gravity from IMU accel data
        
        
        pid_accel_arr = np.array([pid_accel[0], pid_accel[1], pid_accel[2]])
        #taking current yaw position, goal linear accel, get the goal roll and pitch angles. Set limit to be +/-20deg to maintain small angle approximation
        
        goal_orient_if = self.goal_lin_accel2ang_pos(self.drone_orient_euler[2], pid_accel_arr, U1, self.MASS)
        #goal_orient_if = np.array([0*(np.pi/180), 2*(np.pi/180)])
        
        # if (goal_orient_if[0] >= self.BOUND_ROLL_POS_CTRL):
        #     goal_orient_if[0] = self.BOUND_ROLL_POS_CTRL
        # elif (goal_orient_if[0] <= self.BOUND_ROLL_POS_CTRL*-1):
        #     goal_orient_if[0] = self.BOUND_ROLL_POS_CTRL*-1

        # if (goal_orient_if[1] >= self.BOUND_PITCH_POS_CTRL):
        #     goal_orient_if[1] = self.BOUND_PITCH_POS_CTRL
        # elif (goal_orient_if[1] <= self.BOUND_PITCH_POS_CTRL*-1):
        #     goal_orient_if[1] = self.BOUND_PITCH_POS_CTRL*-1
        
        #NOTE: Second method of calculating goal orientation, not currently used in the controller application
        # goal_orient_if2 = self.goal_lin_accel2ang_pos2(self.drone_orient_euler[2], self.goal_lin_accel_if, v1, self.GRAVITY)

        #Calculate the derivative of the angular position (roll and pitch) wrt time
        # goal_ang_vel = (goal_orient_if - self.drone_orient_euler[0:1])/(CTRL_DT['Ts']) #divide by the time expected to complete one control loop cycle
        
        #////////////////////////////////////////////////////////////////////////
        #NOTE: As done in the behavioral controller, set the goal angular velocity to be a maximum with a weight
        goal_ang_vel = np.zeros([2])
        ang_dist_roll =  goal_orient_if[0] - self.drone_orient_euler[0]
        if (ang_dist_roll > 0): #determine the direction of the angular position error to determine the direction the angular velocity needs to be
            #Determine the weight used for maintain orientation behavior (linearly adjusts)
            if (ang_dist_roll > self.BOUND_ANG_POS_RP): 
                roll_ang_weight = self.W_RP
            else:
                roll_ang_weight = self.W_RP*np.divide(ang_dist_roll, self.BOUND_ANG_POS_RP)
            #self.get_logger().info(f'yaw_ang_weight: {yaw_ang_weight}')
            goal_ang_vel[0] = (roll_ang_weight*self.MAX_ANG_VEL_RP)
        else:
            #Determine the weight used for maintain orientation behavior (linearly adjusts)
            if (ang_dist_roll < self.BOUND_ANG_POS_RP*-1): 
                roll_ang_weight = self.W_RP*-1
            else:
                roll_ang_weight = self.W_RP*np.divide(ang_dist_roll, self.BOUND_ANG_POS_RP)
            #self.get_logger().info(f'yaw_ang_weight: {yaw_ang_weight}')
            goal_ang_vel[0] = (roll_ang_weight*self.MAX_ANG_VEL_RP)

        ang_dist_pitch =  goal_orient_if[1] - self.drone_orient_euler[1]
        if (ang_dist_pitch > 0):
            #Determine the weight used for maintain orientation behavior (linearly adjusts)
            if (ang_dist_pitch > self.BOUND_ANG_POS_RP): 
                pitch_ang_weight = self.W_RP
            else:
                pitch_ang_weight = self.W_RP*np.divide(ang_dist_pitch, self.BOUND_ANG_POS_RP)
            #self.get_logger().info(f'yaw_ang_weight: {yaw_ang_weight}')
            goal_ang_vel[1] = (pitch_ang_weight*self.MAX_ANG_VEL_RP)
        else:
            #Determine the weight used for maintain orientation behavior (linearly adjusts)
            if (ang_dist_pitch < self.BOUND_ANG_POS_RP*-1): 
                pitch_ang_weight = self.W_RP*-1
            else:
                pitch_ang_weight = self.W_RP*np.divide(ang_dist_pitch, self.BOUND_ANG_POS_RP)
            #self.get_logger().info(f'yaw_ang_weight: {yaw_ang_weight}')
            goal_ang_vel[1] = (pitch_ang_weight*self.MAX_ANG_VEL_RP)
        

        #/////////////////////////////////////////////////////////////////////////

        #/////////////////////////////////////////////////////////////// NOTE: Calculation for PID Roll (PHI) control
        phi_Kp = POLES['roll_Kp']
        phi_Ki = POLES['roll_Ki']
        phi_Kd = POLES['roll_Kd']

        #calc phi angle error
        phi_err = goal_orient_if[0] - self.drone_orient_euler[0]

        #Derivative of goal phi (roll) position and current drone phi (roll) position. 
        
        phi_err_derivative =  goal_ang_vel[0] - drone_ang_vel_if[0]
        
        #NOTE: INTEGRAL CALCULATION FOR PID
        if (np.absolute(swarm_goal_pos[2] - self.goal_pos_if[2]) <= self.BOUND_ALT_POS_CTRL):
            if (np.absolute(goal_orient_if[0] - self.drone_orient_euler[0]) <= self.BOUND_ROLL_POS_CTRL): #Prevent integral wind-up while drone is constantly adjusting altitude
                #Calculate incremental integral between the last iteration and this current one
                phi_err_integral_dt = np.array([[(timestamp - self.past_timestamp)*(np.asscalar(self.past_phi_err + phi_err)*0.5)]])
            else:
                phi_err_integral_dt = np.array([[0.0]])
        else:
            phi_err_integral_dt = np.array([[0.0]])

        (arr_col_size, arr_row_size) = self.phi_err_integral_array.shape
        if (arr_row_size < self.PID_INTEGRAL_WINDOW_SIZE):
            # self.get_logger().info(f'IF self.phi_err_integral_array: {self.phi_err_integral_array}')
            # self.get_logger().info(f'IF phi_err_integral_dt: {phi_err_integral_dt}')
            self.phi_err_integral_array = np.concatenate((self.phi_err_integral_array, phi_err_integral_dt), axis=1)
             
        else:
            # self.get_logger().info(f'ELSE self.phi_err_integral_array: {self.phi_err_integral_array}')
            # self.get_logger().info(f'ELSE phi_err_integral_dt: {phi_err_integral_dt}')

            self.phi_err_integral_array = np.concatenate((self.phi_err_integral_array, phi_err_integral_dt), axis=1)
            
            self.phi_err_integral_array = np.delete(self.phi_err_integral_array, 0, 1)
        
        phi_err_integral = np.sum(self.phi_err_integral_array, axis=1)    

        if (np.absolute(swarm_goal_pos[2] - self.goal_pos_if[2]) <= self.BOUND_ALT_POS_CTRL): #Set heading controller to start once altitude has been reached.
 
            v2 =  phi_Kp*phi_err + phi_Kd*phi_err_derivative + phi_Ki*phi_err_integral

            #feedback control
            J1 = self.J/self.Ixx
            I1 = (self.Iyy - self.Izz)/self.Ixx
            control_out_U2_bf = self.Ixx*(-(I1*drone_ang_vel_bf[1]*drone_ang_vel_bf[2]) - J1*drone_ang_vel_bf[1]*past_omega_sum + v2)
            control_out_U2_if = self.Ixx*(-(I1*drone_ang_vel_if[1]*drone_ang_vel_if[2]) - J1*drone_ang_vel_if[1]*past_omega_sum + v2)
            control_out_U2 = control_out_U2_bf
        else:
            control_out_U2_if = 0.0
            control_out_U2_bf = 0.0
            control_out_U2 = 0.0 #Set to have zero output, for no control effect until altitude has been reached
            v2 = 0.0 #set to zero to show on csv data logging sheet
        
        # v2 =  phi_Kp*phi_err + phi_Kd*phi_err_derivative + phi_Ki*phi_err_integral

        # #feedback control
        # J1 = self.J/self.Ixx
        # I1 = (self.Iyy - self.Izz)/self.Ixx
        # control_out_U2_bf = self.Ixx*(-(I1*drone_ang_vel_bf[1]*drone_ang_vel_bf[2]) - J1*drone_ang_vel_bf[1]*past_omega_sum + v2)
        # control_out_U2_if = self.Ixx*(-(I1*drone_ang_vel_if[1]*drone_ang_vel_if[2]) - J1*drone_ang_vel_if[1]*past_omega_sum + v2)
        # control_out_U2 = control_out_U2_bf

        #/////////////////////////////////////////////////////////////// NOTE: Calculation for PID Pitch (THETA) control
        theta_Kp = POLES['pitch_Kp']
        theta_Ki = POLES['pitch_Ki']
        theta_Kd = POLES['pitch_Kd']

        #calc phi angle error
        theta_err = goal_orient_if[1] - self.drone_orient_euler[1]

        #Derivative of goal phi (roll) position and current drone phi (roll) position. 
        
        theta_err_derivative =  goal_ang_vel[1] - drone_ang_vel_if[1]
        
        #NOTE: INTEGRAL CALCULATION FOR PID
        if (np.absolute(swarm_goal_pos[2] - self.goal_pos_if[2]) <= self.BOUND_ALT_POS_CTRL):
            if (np.absolute(goal_orient_if[1] - self.drone_orient_euler[1]) <= self.BOUND_ROLL_POS_CTRL): #Prevent integral wind-up while drone is constantly adjusting altitude
                #Calculate incremental integral between the last iteration and this current one
                theta_err_integral_dt = np.array([[(timestamp - self.past_timestamp)*(np.asscalar(self.past_theta_err + theta_err)*0.5)]])
            else:
                theta_err_integral_dt = np.array([[0.0]])
        else:
            theta_err_integral_dt = np.array([[0.0]])

        (arr_col_size, arr_row_size) = self.theta_err_integral_array.shape
        if (arr_row_size < self.PID_INTEGRAL_WINDOW_SIZE):
            # self.get_logger().info(f'IF self.theta_err_integral_array: {self.theta_err_integral_array}')
            # self.get_logger().info(f'IF theta_err_integral_dt: {theta_err_integral_dt}')
            self.theta_err_integral_array = np.concatenate((self.theta_err_integral_array, theta_err_integral_dt), axis=1)
            
        else:
            # self.get_logger().info(f'ELSE self.theta_err_integral_array: {self.theta_err_integral_array}')
            # self.get_logger().info(f'ELSE theta_err_integral_dt: {theta_err_integral_dt}')

            self.theta_err_integral_array = np.concatenate((self.theta_err_integral_array, theta_err_integral_dt), axis=1)
            
            self.theta_err_integral_array = np.delete(self.theta_err_integral_array, 0, 1)
        
        theta_err_integral = np.sum(self.theta_err_integral_array, axis=1)    

        # if (np.absolute(swarm_goal_pos[2] - self.goal_pos_if[2]) <= self.BOUND_ALT_POS_CTRL): #Set heading controller to start once altitude has been reached.
 
        #     v3 =  theta_Kp*theta_err + theta_Kd*theta_err_derivative + theta_Ki*theta_err_integral

        #     #feedback control
        #     J2 = self.J/self.Iyy
        #     I2 = (self.Izz - self.Ixx)/self.Iyy
        #     control_out_U3_bf = self.Iyy*(-(I2*drone_ang_vel_bf[0]*drone_ang_vel_bf[2]) - J2*drone_ang_vel_bf[0]*past_omega_sum + v3)
        #     control_out_U3_if = self.Iyy*(-(I2*drone_ang_vel_if[0]*drone_ang_vel_if[2]) - J2*drone_ang_vel_if[0]*past_omega_sum + v3)
        #     control_out_U3 = control_out_U3_bf
        #     #control_out_U3 = 0.0 #NOTE: SET TO ZERO FOR TESTING ONLY PHI (ABOUT Y - AXIS) CONTROLLER
        # else:
        #     control_out_U3_if = 0.0
        #     control_out_U3_bf = 0.0
        #     control_out_U3 = 0.0 #Set to have zero output, for no control effect until altitude has been reached
        #     v3 = 0.0 #set to zero to show on csv data logging sheet

        v3 =  theta_Kp*theta_err + theta_Kd*theta_err_derivative + theta_Ki*theta_err_integral

        #feedback control
        J2 = self.J/self.Iyy
        I2 = (self.Izz - self.Ixx)/self.Iyy
        control_out_U3_bf = self.Iyy*(-(I2*drone_ang_vel_bf[0]*drone_ang_vel_bf[2]) - J2*drone_ang_vel_bf[0]*past_omega_sum + v3)
        control_out_U3_if = self.Iyy*(-(I2*drone_ang_vel_if[0]*drone_ang_vel_if[2]) - J2*drone_ang_vel_if[0]*past_omega_sum + v3)
        control_out_U3 = control_out_U3_bf
        #control_out_U3 = 0.0 #NOTE: SET TO ZERO FOR TESTING ONLY PHI (ABOUT Y - AXIS) CONTROLLER


        #//////////////////////////////////////////////////////////////
        #////////////////////////////////////////////////////////////// 
        # List that we want to add as a new row for data collection
        # csv_list = [
        # timestamp,
        # self.goal_pos_if[0], self.goal_pos_if[1], self.goal_pos_if[2],
        # self.drone_pos_if[0], self.drone_pos_if[1], self.drone_pos_if[2],

        # np.asscalar(pid_accel_arr[0]), np.asscalar(pid_accel_arr[1]), np.asscalar(pid_accel_arr[2]),
        # np.asscalar(self.goal_lin_accel_if[0]), np.asscalar(self.goal_lin_accel_if[1]), np.asscalar(self.goal_lin_accel_if[2]),
        # np.asscalar(self.drone_orient_euler[0]*(180/np.pi)), np.asscalar(self.drone_orient_euler[1]*(180/np.pi)), np.asscalar(self.drone_orient_euler[2]*(180/np.pi)),
        # np.asscalar(goal_orient_if[0]*(180/np.pi)), np.asscalar(goal_orient_if[1]*(180/np.pi)),
        # np.asscalar(goal_orient_if2[0]*(180/np.pi)), np.asscalar(goal_orient_if2[1]*(180/np.pi)),

        # phi_err*(180/np.pi),
        # phi_err_derivative*(180/np.pi),
        # phi_err_integral*(180/np.pi),
        # v2,

        # theta_err*(180/np.pi),
        # theta_err_derivative*(180/np.pi),
        # theta_err_integral*(180/np.pi),
        # v3,

        # roll_ang_weight, pitch_ang_weight,
        # np.asscalar(goal_ang_vel[0]*(180/np.pi)), np.asscalar(goal_ang_vel[1]*(180/np.pi)),
        # np.asscalar(drone_ang_vel_bf[0]*(180/np.pi)), np.asscalar(drone_ang_vel_bf[1]*(180/np.pi)), np.asscalar(drone_ang_vel_bf[2]*(180/np.pi)),
        # np.asscalar(drone_ang_vel_if[0]*(180/np.pi)), np.asscalar(drone_ang_vel_if[1]*(180/np.pi)), np.asscalar(drone_ang_vel_if[2]*(180/np.pi)),
        # past_omega_sum,

        # float(control_out_U2), control_out_U2_bf, control_out_U2_if,
        # float(control_out_U3), control_out_U3_bf, control_out_U3_if,
        # float(U1),

        # ]

        # csv_list_header = [
        # "timestamp",
        # "Goal_Pos_IF_X", "Goal_Pos_IF_Y", "Goal_Pos_IF_Z",
        # "GPS_Pos_IF_X", "GPS_Pos_IF_Y", "GPS_Pos_IF_Z",

        # "pid_accel_x", "pid_accel_Y", "pid_accel_Z",
        # "Goal_Accel_IF_X", "Goal_Accel_IF_Y", "Goal_Accel_IF_Z",
        # "Drone_Euler_Orient_X", "Drone_Euler_Orient_Y", "Drone_Euler_Orient_Z",
        # "Goal_Euler_Orient_X", "Goal_Euler_Orient_Y", 
        # "2_Goal_Euler_Orient_X", "2_Goal_Euler_Orient_Y", 

        # "phi_err",
        # "phi_err_derivative",
        # "phi_err_integral",
        # "V2",

        # "theta_err",
        # "theta_err_derivative",
        # "theta_err_integral",
        # "V3",

        # "roll_ang_weight", "pitch_ang_weight",
        # "Goal_Ang_Vel_if_X", "Goal_Ang_Vel_if_Y", 
        # "Drone_Ang_Vel_bf_X", "Drone_Ang_Vel_bf_Y", "Drone_Ang_Vel_bf_Z",
        # "Drone_Ang_Vel_if_X", "Drone_Ang_Vel_if_Y", "Drone_Ang_Vel_if_Z",
        # "Past_Omega_Sum",

        # "control_out_U2", "control_out_U2_bf", "control_out_U2_if",
        # "control_out_U3", "control_out_U3_bf", "control_out_U3_if",
        # "control_out_U1",

        # ]
        
        # directory_csv = os.path.join('/home/cnchano/', self.drone_namespace + '_attitude_ctrl_out.csv') #os.path.join(os.getcwd(), 'src/drone_control/pos_ctrl_out.csv')
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
        # #///////////////////////////////////////////////////////////
        #/////////////////////////////////////////////////////////// NOTE: Save error to for next iteration when calculating PID integral
        #Save current values for the next calculation iteration
        self.past_timestamp = timestamp
        self.past_phi_err = phi_err #roll
        self.past_theta_err = theta_err #pitch
        
        self.past_goal_orient_if = goal_orient_if #goal roll and pitch
        #///////////////////////////////////////////////////////////
        #///////////////////////////////////////////////////////////

        return [control_out_U2, control_out_U3]

    def goal_lin_accel2ang_pos(self, orient_yaw_if, goal_lin_accel_if, U1, MASS):
        """calculate desired roll and pitch angles for the drone to use as the goal based on the desired linear acceleration
        OUTPUT: shape (2,1) goal orientation angles pitch and roll"""
        goal_orient_if = np.zeros([2,1])
        sin_yaw = np.sin(orient_yaw_if) 
        cos_yaw = np.cos(orient_yaw_if)
        accel_X = np.asscalar(goal_lin_accel_if[0]) 
        accel_Y = np.asscalar(goal_lin_accel_if[1])

        if (U1 < MASS):
            # Set U1 = MASS so that Mass/U1 = 1
            #Prevents large number outliers and in the end minimizes the effect 
            goal_orient_if = (1)*np.array([[accel_X*sin_yaw - accel_Y*cos_yaw], [accel_X*cos_yaw + accel_Y*sin_yaw]])
            self.get_logger().info(f'Set U1 equal to MASS... U1 = {U1}')
        else:
            goal_orient_if = (MASS/U1)*np.array([[accel_X*sin_yaw - accel_Y*cos_yaw], [accel_X*cos_yaw + accel_Y*sin_yaw]])
        
        return goal_orient_if
    
    def goal_lin_accel2ang_pos2(self, orient_yaw_if, goal_lin_accel_if, Vz, GRAVITY):
        """calculate desired roll and pitch angles for the drone to use as the goal based on the desired linear acceleration
        OUTPUT: shape (2,1) goal orientation angles pitch and roll"""
        goal_orient_if = np.zeros([2,1])
        sin_yaw = np.sin(orient_yaw_if) 
        cos_yaw = np.cos(orient_yaw_if)
        accel_X = np.asscalar(goal_lin_accel_if[0]) 
        accel_Y = np.asscalar(goal_lin_accel_if[1])
        a = accel_X/(Vz + GRAVITY)
        b = accel_Y/(Vz + GRAVITY)

        goal_orient_theta_if = np.arctan(a*cos_yaw + b*sin_yaw)
        if (orient_yaw_if < (np.pi/4)) or (orient_yaw_if > (3*np.pi/4)):
            goal_orient_phi_if = np.arctan2((np.cos(goal_orient_theta_if)*(np.tan(goal_orient_theta_if)*sin_yaw - b)), cos_yaw)
        else:
            goal_orient_phi_if = np.arctan2((np.cos(goal_orient_theta_if)*(a - np.tan(goal_orient_theta_if)*cos_yaw)), sin_yaw)
        goal_orient_if = np.array([[np.asscalar(goal_orient_phi_if)],[np.asscalar(goal_orient_theta_if)]])
        return goal_orient_if
        
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
        # Relative Rotational matrix that relates body frame angular velocity to inertial frame angular velocity
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

    attitude_controller = AttitudeController()

    rclpy.spin(attitude_controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    attitude_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()