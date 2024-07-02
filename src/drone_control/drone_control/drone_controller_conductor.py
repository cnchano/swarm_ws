import rclpy
from rclpy.node import Node


from drone_common.srv import SimClock
from drone_common.srv import PosControl
from drone_common.srv import AttitudeControl
from drone_common.msg import MotorSpeed
from geometry_msgs.msg import Wrench, Vector3
from .constants import (
    
    DRONE_CONSTANT,
    CTRL_DT)

# Import writer class from csv module
from csv import writer
import os
import numpy as np

class DroneControllerConductor(Node):

    def __init__(self):
        super().__init__('drone_control_conductor')
        self.drone_ctrl_publisher = self.create_publisher(MotorSpeed, 'motor_speed_cmd', 10)
         
        timer_period = CTRL_DT['Ts']  # seconds
        self.timer = self.create_timer(timer_period, self.drone_ctrl_publisher_callback)
        
        self.controller_iter_count = 0

        #CREATE PUBLISHER FOR EACH PROPELLER

        self.drone_ctrl_prop_FR1_publisher = self.create_publisher(Wrench, 'prop_FR1/motor_speed_cmd', 10)
        self.drone_ctrl_prop_FL2_publisher = self.create_publisher(Wrench, 'prop_FL2/motor_speed_cmd', 10)
        self.drone_ctrl_prop_RL3_publisher = self.create_publisher(Wrench, 'prop_RL3/motor_speed_cmd', 10)
        self.drone_ctrl_prop_RR4_publisher = self.create_publisher(Wrench, 'prop_RR4/motor_speed_cmd', 10)
        #//////////////////////////////////

        self.sim_clock_sub_node = rclpy.create_node('sim_clock_sub_node') #create sub-node for gps client
        #Initialize sim_clock Client
        self.sim_clock_client = self.sim_clock_sub_node.create_client(SimClock, 'sim_clock_service')
        while not self.sim_clock_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Sim Clock service not available, waiting again...')
        self.sim_clock_req = SimClock.Request()

        #Initialize Pos Controller Client
        self.pos_ctrl_sub_node = rclpy.create_node('pos_ctrl_sub_node') #create sub-node for position control client
        self.pos_ctrl_client = self.pos_ctrl_sub_node.create_client(PosControl, 'pos_ctrl_service')
        while not self.pos_ctrl_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Position Control service not available, waiting again...')
        self.pos_ctrl_req = PosControl.Request()

        #Initialize Attitude Controller Client
        self.attitude_ctrl_sub_node = rclpy.create_node('attitude_ctrl_sub_node') #create sub-node for attitude control client
        self.attitude_ctrl_client = self.attitude_ctrl_sub_node.create_client(AttitudeControl, 'attitude_ctrl_service')
        while not self.attitude_ctrl_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('attitude Control service not available, waiting again...')
        self.attitude_ctrl_req = AttitudeControl.Request()
        
        #//////////////////////////////////
        #ROS2 Parameter
        #Get drone namespace to extract its own drone_id
        self.declare_parameter('drone_ns')
        self.drone_namespace = self.get_parameter('drone_ns').get_parameter_value().string_value 
        #//////////////////////////////////
        
        
        #Set previous time to initialize at zero sec
        self.prev_ctrl_loop_timestamp = 0.0 #sec
        #initialize control output forces to ensure the values are non-zero but also are at idle rpm for the drone
        #init_omega_sqrd = DRONE_CONSTANT['init_omega_prop']**2 #rad/sec
        self.past_control_out_U2 = 0.0
        self.past_control_out_U3 = 0.0
        self.past_control_out_U4 = 0.0
        
        #Initialize past commanded omega value to zero
        self.past_omega_vect = np.zeros((4,1))

        self.past_omega_sum = 0.0

        #Initialize iteration counter for csv write to initialize file, then later add variables
        self.iter_count = 0.0
        

    def drone_ctrl_publisher_callback(self):
        
        #import timestamp
        clock_data = self.call_sim_clock_service()
        #self.get_logger().info(f'GPS service data: {clock_data}')
        
        #Pass timestamp data
        ctrl_loop_timestamp = clock_data.sim_timestamp
        
        if (ctrl_loop_timestamp > 2.0):
        
            #realtime dt
            rt_ctrl_dt = ctrl_loop_timestamp - self.prev_ctrl_loop_timestamp #sec

            #Get U1 and U4
            pos_ctrl_data = self.call_pos_ctrl_service()

            U1 = pos_ctrl_data.pos_control_out_u1
            U4 = 0.0 #pos_ctrl_data.control_out_u4

            #Get U2 and U3 ONLY if U1 > 40 which based on data is below the average to maintain altitude
            #If the drone is in freefall, then there is not enough force to adjust the attitude with 
            if (U1 > 40):
                attitude_ctrl_data = self.call_attitude_ctrl_service(pos_ctrl_data.pos_control_out_u1, self.past_omega_sum, pos_ctrl_data.v1, pos_ctrl_data.pid_accel)
                U2 = attitude_ctrl_data.control_out_u2
                U3 = attitude_ctrl_data.control_out_u3
            else:
                U2 = 0.0
                U3 = 0.0
            
            
            #////////////////
            #/////////////////
            #///////////////
            if (np.absolute(self.past_omega_vect[0,0]) < 104.7198): #if motor speed below threshold, then set to min Coef of thrust
                coef_thrust_m1 = 0.011681
            elif (np.absolute(self.past_omega_vect[0,0]) > 445.059): #if motor speed above threshold, then set to max Coef of thrust
                coef_thrust_m1 = 0.010348
            elif (np.absolute(self.past_omega_vect[0,0]) <= 157.07965): #if motor speed BELOW threshold, then set to curve fit
                coef_thrust_m1 = -2*(10**-5)*np.asscalar(np.absolute(self.past_omega_vect[0,0])) + 0.0143
            else:
                coef_thrust_m1 = 2*(10**-7)*np.asscalar(np.absolute(self.past_omega_vect[0,0])) + 0.0102
                
            if (np.absolute(self.past_omega_vect[1,0]) < 104.7198):
                coef_thrust_m2 = 0.011681
            elif (np.absolute(self.past_omega_vect[1,0]) > 445.059):
                coef_thrust_m2 = 0.010348
            elif (np.absolute(self.past_omega_vect[1,0]) <= 157.07965):
                coef_thrust_m2 = -2*(10**-5)*np.asscalar(np.absolute(self.past_omega_vect[1,0])) + 0.0143
            else:
                coef_thrust_m2 = 2*(10**-7)*np.asscalar(np.absolute(self.past_omega_vect[1,0])) + 0.0102

            if (np.absolute(self.past_omega_vect[2,0]) < 104.7198):
                coef_thrust_m3 = 0.011681
            elif (np.absolute(self.past_omega_vect[2,0]) > 445.059):
                coef_thrust_m3 = 0.010348
            elif (np.absolute(self.past_omega_vect[2,0]) <= 157.07965):
                coef_thrust_m3 = -2*(10**-5)*np.asscalar(np.absolute(self.past_omega_vect[2,0])) + 0.0143
            else:
                coef_thrust_m3 = 2*(10**-7)*np.asscalar(np.absolute(self.past_omega_vect[2,0])) + 0.0102

            if (np.absolute(self.past_omega_vect[3,0]) < 104.7198):
                coef_thrust_m4 = 0.011681
            elif (np.absolute(self.past_omega_vect[3,0]) > 445.059):
                coef_thrust_m4 = 0.010348
            elif (np.absolute(self.past_omega_vect[3,0]) <= 157.07965):
                coef_thrust_m4 = -2*(10**-5)*np.asscalar(np.absolute(self.past_omega_vect[3,0])) + 0.0143
            else:
                coef_thrust_m4 = 2*(10**-7)*np.asscalar(np.absolute(self.past_omega_vect[3,0])) + 0.0102

            #Set Coef of torque for each motor
            #NOTE: For debugging
            # coef_torque_m1 = 0.001282347
            # coef_torque_m2 = 0.001282347
            # coef_torque_m3 = 0.001282347
            # coef_torque_m4 = 0.001282347

            if (self.past_omega_vect[0,0] < 104.7198): #if motor speed below threshold, then set to min Coef of torque
                coef_torque_m1 = 0.013677
            elif (self.past_omega_vect[0,0] > 445.059): #if motor speed above threshold, then set to max Coef of torque
                coef_torque_m1 = 0.00073
            else:
                coef_torque_m1 = 538.54*(np.asscalar(self.past_omega_vect[0,0])**-2.282)

            if (self.past_omega_vect[1,0] < 104.7198):
                coef_torque_m2 = 0.013677
            elif (self.past_omega_vect[1,0] > 445.059):
                coef_torque_m2 = 0.00073
            else:
                coef_torque_m2 = 538.54*(np.asscalar(self.past_omega_vect[1,0])**-2.282)

            if (self.past_omega_vect[2,0] < 104.7198):
                coef_torque_m3 = 0.013677
            elif (self.past_omega_vect[2,0] > 445.059):
                coef_torque_m3 = 0.00073
            else:
                coef_torque_m3 = 538.54*(np.asscalar(self.past_omega_vect[2,0])**-2.282)

            if (self.past_omega_vect[3,0] < 104.7198):
                coef_torque_m4 = 0.013677
            elif (self.past_omega_vect[3,0] > 445.059):
                coef_torque_m4 = 0.00073
            else:
                coef_torque_m4 = 538.54*(np.asscalar(self.past_omega_vect[3,0])**-2.282)

            
            #//////////////////
            #/////////////////
            #/////////////////
 

            #Calculate commanded motor speeds from control output (units of forces N and moments N-m)
            # Compute the new omegas based on the new U-s

            # ctrl_out_vector_Uc = np.zeros((4,1))
            # ctrl_out_vector_Uc[0,0] = mpc_data.control_out_u1/DRONE_CONSTANT['Ct']
            # ctrl_out_vector_Uc[1,0] = mpc_data.control_out_u2/(DRONE_CONSTANT['Ct']*DRONE_CONSTANT['arm_length'])
            # ctrl_out_vector_Uc[2,0] = mpc_data.control_out_u3/(DRONE_CONSTANT['Ct']*DRONE_CONSTANT['arm_length'])
            # ctrl_out_vector_Uc[3,0] = mpc_data.control_out_u4/DRONE_CONSTANT['Cq']
            
            THRUST_CONST = DRONE_CONSTANT['air_density']*(DRONE_CONSTANT['prop_radius']**4)*(np.pi)
            TORQUE_CONST = DRONE_CONSTANT['air_density']*(DRONE_CONSTANT['prop_radius']**5)*(np.pi)
            ctrl_out_vector_Uc = np.zeros((4,4))
            ctrl_out_vector_Uc[0,:] = np.array([[(U1)/(coef_thrust_m1*THRUST_CONST), (U2)/(coef_thrust_m1*THRUST_CONST), (U3)/(coef_thrust_m1*THRUST_CONST), (U4)/(coef_torque_m1*TORQUE_CONST) ]])
            ctrl_out_vector_Uc[1,:] = np.array([[(U1)/(coef_thrust_m2*THRUST_CONST), (U2)/(coef_thrust_m2*THRUST_CONST), (U3)/(coef_thrust_m2*THRUST_CONST), (U4)/(coef_torque_m2*TORQUE_CONST) ]])
            ctrl_out_vector_Uc[2,:] = np.array([[(U1)/(coef_thrust_m3*THRUST_CONST), (U2)/(coef_thrust_m3*THRUST_CONST), (U3)/(coef_thrust_m3*THRUST_CONST), (U4)/(coef_torque_m3*TORQUE_CONST) ]])
            ctrl_out_vector_Uc[3,:] = np.array([[(U1)/(coef_thrust_m4*THRUST_CONST), (U2)/(coef_thrust_m4*THRUST_CONST), (U3)/(coef_thrust_m4*THRUST_CONST), (U4)/(coef_torque_m4*TORQUE_CONST) ]])
            # self.get_logger().info(f' ctrl_out_vector_Uc : {ctrl_out_vector_Uc}')
            omega_mat_inv =np.zeros((4,4))
            omega_mat_inv[0,:] = np.array([[0.25, 0.0, -0.5, -0.25]])
            omega_mat_inv[1,:] = np.array([[0.25, 0.5, 0.0, 0.25]])
            omega_mat_inv[2,:] = np.array([[0.25, 0.0, 0.5, -0.25]])
            omega_mat_inv[3,:] = np.array([[0.25, -0.5, 0.0, 0.25]])

            omega_sqrd_vect = np.multiply(omega_mat_inv, ctrl_out_vector_Uc)
            # self.get_logger().info(f' MULTIPLY omega_sqrd_vect : {omega_sqrd_vect}')
            omega_sqrd_vect = omega_sqrd_vect.sum(axis=1, dtype='float')
            omega_sqrd_vect = np.array([[omega_sqrd_vect[0]], [omega_sqrd_vect[1]], [omega_sqrd_vect[2]], [omega_sqrd_vect[3]]])
            # self.get_logger().info(f' SUM omega_sqrd_vect : {omega_sqrd_vect}')

            if (omega_sqrd_vect[0] <= 0.0):
                omega_sqrd_vect[0] = 0.0001
            if (omega_sqrd_vect[1] <= 0.0):
                omega_sqrd_vect[1] = 0.0001
            if (omega_sqrd_vect[2] <= 0.0):
                omega_sqrd_vect[2] = 0.0001
            if (omega_sqrd_vect[3] <= 0.0):
                omega_sqrd_vect[3] = 0.0001


            
            # #self.get_logger().info(f' ctrl_out_vector_Uc : {ctrl_out_vector_Uc}')

            # self.get_logger().info(f' omega_sqrd_vect : {omega_sqrd_vect}')


            if omega_sqrd_vect[0]<0 or omega_sqrd_vect[1]<0 or omega_sqrd_vect[2]<0 or omega_sqrd_vect[3]<0:
                print("You can't take a square root of a negative number")
                print("The problem might be that the trajectory is too chaotic or it might have large discontinuous jumps")
                print("Try to make a smoother trajectory without discontinuous jumps")
                print("Other possible causes might be values for variables such as Ts, hz, innerDyn_length, px, py, pz")
                #///////////////////////////////////////////////////////////////////////////////
                # #////////////////////////////////////////////////////////////////////////////////
                # # List that we want to add as a new row for data collection
                # csv_list = [
                # ctrl_loop_timestamp, 
                # omega_sqrd_vect[0], omega_sqrd_vect[1], omega_sqrd_vect[2], omega_sqrd_vect[3], 
                # "NaN", "NaN", "NaN", "NaN", 
                # self.past_omega_vect[0], self.past_omega_vect[1], self.past_omega_vect[2], self.past_omega_vect[3],
                # # self.pos_ctrl_out_phi_ref,  self.pos_ctrl_out_theta_ref, self.pos_ctrl_out_psi_ref, 
                # coef_thrust_m1, #coef_thrust_m2, coef_thrust_m3, coef_thrust_m4, 
                # coef_torque_m1, #coef_torque_m2, coef_torque_m3, coef_torque_m4
                # thrust[0], thrust[1], thrust[2], thrust[3],
                # torque[0], torque[1], torque[2], torque[3],
                # ]
                # # self.behavior_ctrl_goal_pos_if[0], self.behavior_ctrl_goal_pos_if[1], self.behavior_ctrl_goal_pos_if[2], self.behavior_ctrl_goal_lin_vel_if[0], self.behavior_ctrl_goal_lin_vel_if[1], self.behavior_ctrl_goal_lin_vel_if[2], self.behavior_ctrl_goal_lin_accel_if[2], \
                # # self.pos_ctrl_imu_drone_lin_vel_if[0], self.pos_ctrl_imu_drone_lin_vel_if[1], self.pos_ctrl_imu_drone_lin_vel_if[2], self.pos_ctrl_imu_drone_lin_accel_if[0], self.pos_ctrl_imu_drone_lin_accel_if[1], self.pos_ctrl_imu_drone_lin_accel_if[2], \
                # # self.pos_ctrl_est_drone_lin_accel_if[0], self.pos_ctrl_est_drone_lin_accel_if[1], self.pos_ctrl_est_drone_lin_accel_if[2], \
                # # self.pos_ctrl_drone_pos_if[0], self.pos_ctrl_drone_pos_if[1], self.pos_ctrl_drone_pos_if[2]]

                # csv_list_header = [
                # "Sim Clock",  
                # "Omega1^2", "Omega2^2", "Omega3^2", "Omega4^2", 
                # "Omega1", "Omega2", "Omega3", "Omega4",
                # "Past_Omega1", "Past_Omega2", "Past_Omega3", "Past_Omega4",
                # # "phi_ref", "theta_ref", "psi_ref", 
                # "Coef_thrust_m1", #"Coef_thrust_m2", "Coef_thrust_m3", "Coef_thrust_m4", 
                # "Coef_torque_m1", #"Coef_torque_m2", "Coef_torque_m3", "Coef_torque_m4",
                # "thrust[0]", "thrust[1]", "thrust[2]", "thrust[3]",
                # "torque[0]", "torque[1]", "torque[2]", "torque[3]",
                # ]
                # # "Goal_Pos_IF_X", "Goal_Pos_IF_Y", "Goal_Pos_IF_Z", "Goal_Vel_IF_X", "Goal_Vel_IF_Y", "Goal_Vel_IF_Z", \
                # # "IMU_PC_Vel_IF_X", "IMU_PC_Vel_IF_Y", "IMU_PC_Vel_IF_Z", "IMU_PC_Accel_IF_X", "IMU_PC_Accel_IF_Y", "IMU_PC_Accel_IF_Z",\
                # # "PC_Est_Accel_IF_X", "PC_Est_Accel_IF_Y", "PC_Est_Accel_IF_Z"]

                # directory_csv = '/home/cnchano/drone_conductor_out.csv' #os.path.join(os.getcwd(), 'src/drone_control/drone_conductor_out.csv')
                # # self.get_logger().info(f'Getting csv file path: {directory_csv}...')

                # # Open our existing CSV file in append mode
                # # Create a file object for this file
                # with open(directory_csv, 'a') as f_object:
                
                #     # Pass this file object to csv.writer()
                #     # and get a writer object
                #     writer_object = writer(f_object)
                
                #     # Pass the list as an argument into
                #     # the writerow()
                #     writer_object.writerow(csv_list)
                
                #     # Close the file object
                #     f_object.close()

                # #///////////////////////////////////////////////////////////////
                #////////////////////////////////////////////////////////////////////////////////
                exit()
            else:
                omega_vect = np.sqrt(omega_sqrd_vect)
                # self.get_logger().info(f' omega_vect : {omega_vect}')
                # omega_vect = np.array([[np.asscalar(omega_vect[0])], [np.asscalar(omega_vect[1])], [np.asscalar(omega_vect[2])], [np.asscalar(omega_vect[3])]])
                #omega_vect = np.array([[250],[250],[250],[250]])
                prop_dir = np.array([[-1],[1],[-1],[1]])
                
                omega_vect = np.multiply(omega_vect, prop_dir)
                # self.get_logger().info(f' omega_vect : {omega_vect}')


                thrust = np.zeros([4,1])
                torque = np.zeros([4,1])
 
                for i in range(4):
                    
                    thrust_i = ((0.0002*(np.asscalar(omega_vect[i,0])**2)) - 0.0083*np.absolute(np.asscalar(omega_vect[i,0])) + 1.0069) 
                    # thrust_i = ((0.0002*(np.asscalar(omega_vect[i])**2)) - 0.0083*np.absolute(np.asscalar(omega_vect[i])) + 1.0069)
                    # self.get_logger().info(f'thrust_i: {thrust_i}')
                    thrust[i] = np.array([[thrust_i]])

                    if (np.absolute(omega_vect[i,0]) < 104.7198):
                        coef_torque_min = 0.013677
                        torque_i = (np.copysign(coef_torque_min*(1.2041*0.223459*0.266701*((np.absolute(np.asscalar(omega_vect[i,0]))*0.266701)**2)), np.asscalar(omega_vect[i,0])))
                    elif (np.absolute(omega_vect[i,0])  > 445.059):
                        coef_torque_max = 0.00073
                        torque_i = (np.copysign(coef_torque_max*(1.2041*0.223459*0.266701*((np.absolute(np.asscalar(omega_vect[i,0]))*0.266701)**2)), np.asscalar(omega_vect[i,0])))
                    else:
                        torque_i = (np.copysign((538.54*(np.absolute(np.asscalar(omega_vect[i,0]))**-2.282))*(1.2041*0.223459*0.266701*((np.absolute(np.asscalar(omega_vect[i,0]))*0.266701)**2)), np.asscalar(omega_vect[i,0])))
                    # torque_i = (np.copysign(100*(538.54*(np.absolute(np.asscalar(omega_vect[i]))**-2.282))*(1.2041*0.223459*0.266701*((np.absolute(np.asscalar(omega_vect[i]))*0.266701)**2)), np.asscalar(omega_vect[i])))
                    torque[i] = np.array([[torque_i]])


            # if (ctrl_loop_timestamp >= 35.0 and self.drone_namespace == 'drone_1'):
            # #     thrust[0,0] = thrust[0,0]*0.2
            # #     thrust[1,0] = thrust[1,0]*0.2
            #     thrust[0,0] = 0.0
            #     thrust[1,0] = 0.0
            #     thrust[2,0] = 0.0
            #     thrust[3,0] = 0.0
                

            #////////////////////////////////////////////////////////////////////////////////
            prop_msg = Wrench()
            prop_msg.force = Vector3(x=0.0, y=0.0, z=float(thrust[0,0]))
            self.drone_ctrl_prop_FR1_publisher.publish(prop_msg)

            prop_msg.force = Vector3(x=0.0, y=0.0, z=float(thrust[1,0]))
            self.drone_ctrl_prop_FL2_publisher.publish(prop_msg)
            
            prop_msg.force = Vector3(x=0.0, y=0.0, z=float(thrust[2,0]))
            self.drone_ctrl_prop_RL3_publisher.publish(prop_msg)

            prop_msg.force = Vector3(x=0.0, y=0.0, z=float(thrust[3,0]))
            self.drone_ctrl_prop_RR4_publisher.publish(prop_msg)

            #////////////////////////////////////////////////////////////////////////////////
            ns = "/" + self.drone_namespace + "/"
            msg = MotorSpeed()
            msg.name = [ns + "prop_FR1", ns + "prop_FL2", ns + "prop_RL3", ns + "prop_RR4"] #names for eah propeller link in the xacro file from the drone description
            msg.body_name = ["drone_body"]
 
            msg.torque_body_name = [ns + "torque_FR1", ns + "torque_FL2", ns + "torque_RL3", ns + "torque_RR4"]
            msg.body_torque = [float(torque[0]), float(torque[1]), float(torque[2]), float(torque[3])]

            msg.velocity = [float(omega_vect[0,0]), float(omega_vect[1,0]), float(omega_vect[2,0]), float(omega_vect[3,0])]
            # msg.velocity = [float(omega_vect[0]), float(omega_vect[1]), float(omega_vect[2]), float(omega_vect[3])]
            
            # if (self.drone_namespace == 'drone_1'): #NOTE: (test_var) For testing if other drones driven by other drone controllers
            #     self.drone_ctrl_publisher.publish(msg)
            # self.get_logger().info(f'Publishing motor speed ("prop_FR1", "prop_FL2", "prop_RL3", "prop_RR4"): {msg.velocity}')
            
            #Pass current iteration's motor speed command output for calculating coefficients of thrust and torque
            self.past_omega_vect = omega_vect
            # self.get_logger().info(f' past_omega_vect : {self.past_omega_vect}')
            self.past_omega_sum = np.sum(omega_vect, axis=0)
            # self.get_logger().info(f' omega_sum : {self.past_omega_sum}')

            #////////////////////////////////////////////////////////////////////////////////
            #///////////////////////////////////////////////////////////////////////////////
            # List that we want to add as a new row for data collection
            csv_list = [
            ctrl_loop_timestamp, 
            ctrl_out_vector_Uc[0,0], ctrl_out_vector_Uc[0,1], ctrl_out_vector_Uc[0,2], ctrl_out_vector_Uc[0,3],
            omega_sqrd_vect[0], omega_sqrd_vect[1], omega_sqrd_vect[2], omega_sqrd_vect[3], 
            omega_vect[0], omega_vect[1], omega_vect[2], omega_vect[3], 
            self.past_omega_vect[0], self.past_omega_vect[1], self.past_omega_vect[2], self.past_omega_vect[3],
            # self.pos_ctrl_out_phi_ref,  self.pos_ctrl_out_theta_ref, self.pos_ctrl_out_psi_ref, 
            coef_thrust_m1, #coef_thrust_m2, coef_thrust_m3, coef_thrust_m4, 
            coef_torque_m1, #coef_torque_m2, coef_torque_m3, coef_torque_m4
            thrust[0], thrust[1], thrust[2], thrust[3],
            torque[0], torque[1], torque[2], torque[3],
            ]
            # self.behavior_ctrl_goal_pos_if[0], self.behavior_ctrl_goal_pos_if[1], self.behavior_ctrl_goal_pos_if[2], self.behavior_ctrl_goal_lin_vel_if[0], self.behavior_ctrl_goal_lin_vel_if[1], self.behavior_ctrl_goal_lin_vel_if[2], self.behavior_ctrl_goal_lin_accel_if[2], \
            # self.pos_ctrl_imu_drone_lin_vel_if[0], self.pos_ctrl_imu_drone_lin_vel_if[1], self.pos_ctrl_imu_drone_lin_vel_if[2], self.pos_ctrl_imu_drone_lin_accel_if[0], self.pos_ctrl_imu_drone_lin_accel_if[1], self.pos_ctrl_imu_drone_lin_accel_if[2], \
            # self.pos_ctrl_est_drone_lin_accel_if[0], self.pos_ctrl_est_drone_lin_accel_if[1], self.pos_ctrl_est_drone_lin_accel_if[2], \
            # self.pos_ctrl_drone_pos_if[0], self.pos_ctrl_drone_pos_if[1], self.pos_ctrl_drone_pos_if[2]]

            csv_list_header = [
            "Sim Clock",
            "ctrl_out_vector_Uc_1", "ctrl_out_vector_Uc_2", "ctrl_out_vector_Uc_3", "ctrl_out_vector_Uc_4",
            "Omega1^2", "Omega2^2", "Omega3^2", "Omega4^2", 
            "Omega1", "Omega2", "Omega3", "Omega4",
            "Past_Omega1", "Past_Omega2", "Past_Omega3", "Past_Omega4",
            # "phi_ref", "theta_ref", "psi_ref", 
            "Coef_thrust_m1", #"Coef_thrust_m2", "Coef_thrust_m3", "Coef_thrust_m4", 
            "Coef_torque_m1", #"Coef_torque_m2", "Coef_torque_m3", "Coef_torque_m4",
            "thrust[0]", "thrust[1]", "thrust[2]", "thrust[3]",
            "torque[0]", "torque[1]", "torque[2]", "torque[3]",
            ]
            # "Goal_Pos_IF_X", "Goal_Pos_IF_Y", "Goal_Pos_IF_Z", "Goal_Vel_IF_X", "Goal_Vel_IF_Y", "Goal_Vel_IF_Z", \
            # "IMU_PC_Vel_IF_X", "IMU_PC_Vel_IF_Y", "IMU_PC_Vel_IF_Z", "IMU_PC_Accel_IF_X", "IMU_PC_Accel_IF_Y", "IMU_PC_Accel_IF_Z",\
            # "PC_Est_Accel_IF_X", "PC_Est_Accel_IF_Y", "PC_Est_Accel_IF_Z"]

            directory_csv = os.path.join('/home/cnchano/', self.drone_namespace + '_drone_conductor_out.csv') #os.path.join(os.getcwd(), 'src/drone_control/drone_conductor_out.csv')
            # self.get_logger().info(f'Getting csv file path: {directory_csv}...')
            if (self.iter_count == 0.0): #self.prev_ctrl_loop_timestamp is initialized at zero in the initialization function, therefore the first run of the script should have this value equal to zero
                self.get_logger().info(f'creating Drone Conductor csv file...')
                with open(directory_csv, 'w', newline='') as f_object:
                    writer_object = writer(f_object)
                    writer_object.writerow(csv_list_header)
                    writer_object.writerow(csv_list)
                    # Close the file object
                    f_object.close()
                    self.iter_count += 1
                self.get_logger().info(f'Finished first write to csv file...')

            else:
                # Open our existing CSV file in append mode
                # Create a file object for this file
                with open(directory_csv, 'a') as f_object:
                
                    # Pass this file object to csv.writer()
                    # and get a writer object
                    writer_object = writer(f_object)
                
                    # Pass the list as an argument into
                    # the writerow()
                    writer_object.writerow(csv_list)
                
                    # Close the file object
                    f_object.close()
            #////////////////////////////////////////////////////////////////////////////////
            #////////////////////////////////////////////////////////////////////////////////
            # #/////////////////////////////////////////////////////////////// 
            # #NOTE: Count iterations of the controller made
            # if (self.controller_iter_count == 3):
            #     self.controller_iter_count = 0
            # else:
            #     self.controller_iter_count += 1
            # #///////////////////////////////////////////////////////////////

        #Pass timestamp data
        self.prev_ctrl_loop_timestamp = ctrl_loop_timestamp 
        
    def call_sim_clock_service(self):

        self.sim_clock_future = self.sim_clock_client.call_async(self.sim_clock_req)
        rclpy.spin_until_future_complete(self.sim_clock_sub_node, self.sim_clock_future)
        return self.sim_clock_future.result()

    def call_pos_ctrl_service(self):

        
        self.pos_ctrl_req.drone_name = 'drone1'

        self.pos_ctrl_future = self.pos_ctrl_client.call_async(self.pos_ctrl_req)
        rclpy.spin_until_future_complete(self.pos_ctrl_sub_node, self.pos_ctrl_future)
        return self.pos_ctrl_future.result()
    
    def call_attitude_ctrl_service(self, U1, past_omega_sum, v1, pid_accel):

        self.attitude_ctrl_req.control_out_u1 = float(U1)
        self.attitude_ctrl_req.past_omega_sum = float(past_omega_sum)
        self.attitude_ctrl_req.v1 = float(v1)
        self.attitude_ctrl_req.pid_accel = pid_accel

        self.attitude_ctrl_future = self.attitude_ctrl_client.call_async(self.attitude_ctrl_req)
        rclpy.spin_until_future_complete(self.attitude_ctrl_sub_node, self.attitude_ctrl_future)
        return self.attitude_ctrl_future.result()



def main(args=None):
    rclpy.init(args=args)

    drone_control_conductor = DroneControllerConductor()

    rclpy.spin(drone_control_conductor)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    drone_control_conductor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()