

import rclpy
from rclpy.node import Node

import numpy as np
# Import writer class from csv module
from csv import writer
import os

from drone_common.srv import VelCalc
from drone_common.srv import SimClock
from drone_common.srv import GPStoENU
from .constants import (
    POS_CONTROL,
    DRONE_CONSTANT,
    MPC_CONTROL,
    CTRL_DT,
    FILTER_CONSTANT)

class Velocity_Serv(Node):

    def __init__(self):
        super().__init__('velocity_serv')

        #Service to provide GPS coordinates of drone in cartesian coordinates against a reference location
        self.srv = self.create_service(VelCalc, 'vel_calc_service', self.vel_serv_callback)
        
        timer_period = CTRL_DT['vel_Ts']  # seconds/ set to run faster than the controller loop iteration time
        self.timer = self.create_timer(timer_period, self.velocity_calc_callback)

        self.sim_clock_sub_node = rclpy.create_node('sim_clock_sub_node') #create sub-node for gps client
        #Initialize sim_clock Client
        self.sim_clock_client = self.sim_clock_sub_node.create_client(SimClock, 'sim_clock_service')
        while not self.sim_clock_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.sim_clock_req = SimClock.Request()

        self.gps_sub_node = rclpy.create_node('gps_sub_node') #create sub-node for gps client
        #Initialize GPS Client
        self.gps_client = self.gps_sub_node.create_client(GPStoENU, 'gps_WGS84_to_enu_service')
        while not self.gps_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.gps_req = GPStoENU.Request()

        #self.drone_pos_if = np.zeros([3, 1])
        self.past_drone_pos_if = None #Set to null so that it will later be updated by the initial position of the drone
        self.past_timestamp = 0.0
        self.drone_vel_if = np.zeros([3, 1])

        #Initialize iteration counter for csv write to initialize file, then later add variables
        self.iter_count = 0.0

        #Initialize variables for moving average of velocity
        self.AVG_WINDOW_SIZE = FILTER_CONSTANT['velocity_moving_avg_window']
        self.drone_vel_matrix = np.array([[0.0],[0.0],[0.0]])
        self.avg_drone_vel_if = np.array([[0.0],[0.0],[0.0]])

    def vel_serv_callback(self, request, response):
        response.drone_vel_if = [float(self.drone_vel_if[0]), float(self.drone_vel_if[1]), float(self.drone_vel_if[2])]
        return response
    
    def velocity_calc_callback(self):

        drone_pos_if = np.zeros([3, 1])

        #import IMU data as client/ convert IMU data WRT inertial frame
        # imu_data = self.call_imu_service()
        #self.get_logger().info(f'IMU service
        # data: {imu_data}')
        
        #import GPS data as client/ convert IMU data WRT inertial frame
        gps_data = self.call_gps_service()
        #self.get_logger().info(f'GPS service data: {gps_data}')
        #Pass gps data
        drone_pos_if[0] = gps_data.de #x-axis
        drone_pos_if[1] = gps_data.dn #y-axis
        drone_pos_if[2] = gps_data.du #z-axis
        # self.get_logger().info(f'Current position: {drone_pos_if}')
        #ensure that the past position is initialized to the same position as the current drone position
        #if (np.any(self.past_drone_pos_if == None)):
        # self.get_logger().info(f'Iteration COUNT: {self.iter_count}')
        if (self.iter_count == 0.0):
            self.past_drone_pos_if = drone_pos_if
            # self.get_logger().info(f'Past position INITIALIZED: {self.past_drone_pos_if}')
        #import timestamp
        clock_data = self.call_sim_clock_service()
        #self.get_logger().info(f'GPS service data: {clock_data}')
    
        #Pass timestamp data
        timestamp = clock_data.sim_timestamp
        #Calculate time since last position update
        delta_time = timestamp - self.past_timestamp
        #Calculate differential position
        # self.get_logger().info(f'Past position: {self.past_drone_pos_if}')
        delta_drone_pos_if = drone_pos_if - self.past_drone_pos_if
        # self.get_logger().info(f'Delta position: {delta_drone_pos_if}')
        # self.get_logger().info(f'delta_time: {delta_time}')
        #since clock update rate is so slow, DO NOT UPDATE UNTIL DELTA TIME IS GREATER THAN ZERO
        # check if it is updated to ensure there are no singularities
        if (delta_time != 0.0):
            self.drone_vel_if = delta_drone_pos_if/delta_time
            # self.get_logger().info(f'Velocity: {self.drone_vel_if}')
            
            #/////////////////////////
            #NOTE: MOVING AVERAGE FOR LINEAR VELOCITY
            (arr_col_size, arr_row_size) = self.drone_vel_matrix.shape
            if (arr_row_size < self.AVG_WINDOW_SIZE):
                self.drone_vel_matrix = np.append(self.drone_vel_matrix, np.array([[self.drone_vel_if[0,0]],[self.drone_vel_if[1,0]],[self.drone_vel_if[2,0]]]), axis=1)
                
            else:
                self.drone_vel_matrix = np.append(self.drone_vel_matrix, np.array([[self.drone_vel_if[0,0]],[self.drone_vel_if[1,0]],[self.drone_vel_if[2,0]]]), axis=1)
                
                self.drone_vel_matrix = np.delete(self.drone_vel_matrix, 0, 1)
                
            kernel_array = np.ones(self.AVG_WINDOW_SIZE)/self.AVG_WINDOW_SIZE
            vel_arr_conv_x = np.convolve(self.drone_vel_matrix[0,:], kernel_array, mode='valid')
            vel_arr_conv_y = np.convolve(self.drone_vel_matrix[1,:], kernel_array, mode='valid')
            vel_arr_conv_z = np.convolve(self.drone_vel_matrix[2,:], kernel_array, mode='valid')
            
            self.avg_drone_vel_if = np.array([[vel_arr_conv_x[-1]],[vel_arr_conv_y[-1]],[vel_arr_conv_z[-1]]])
            #/////////////////////////

        #ELSE don't update anything
        
        # self.past_timestamp = timestamp
        # self.past_drone_pos_if = self.drone_pos_if
        # self.drone_vel_if = delta_drone_pos_if/delta_time
        # self.get_logger().info(f'Velocity: {self.drone_vel_if}')
        

        #//////////////////////////////////////////////////////////////
        #NOTE: WRITING TO CSV LIST IS REQUIRED TO PROPERLY CONTROL THE DRONE SINCE IT EFFECTS TIMING OF THE SYSTEM
        #//////////////////////////////////////////////////////////////
        # List that we want to add as a new row for data collection
        csv_list = [ 
        timestamp, self.past_timestamp, delta_time,
        drone_pos_if[0], drone_pos_if[1], drone_pos_if[2],
        self.past_drone_pos_if[0], self.past_drone_pos_if[1], self.past_drone_pos_if[2],
        delta_drone_pos_if[0], delta_drone_pos_if[1], delta_drone_pos_if[2],
        self.drone_vel_if[0], self.drone_vel_if[1], self.drone_vel_if[2],
        self.avg_drone_vel_if[0], self.avg_drone_vel_if[1], self.avg_drone_vel_if[2],
        
        ]
        csv_list_header = [
        'imu_drone_timestamp', 'past_imu_drone_timestamp', 'delta_time',
        'drone pos_X', 'drone pos_Y', 'drone pos_Z',
        'past_drone pos_X', 'past_drone pos_Y', 'past_drone pos_Z',
        'delta_drone_pos_if[X]', 'delta_drone_pos_if[Y]', 'delta_drone_pos_if[Z]',
        'drone_lin_vel[X]', 'drone_lin_vel[Y]', 'drone_lin_vel[Z]',
        'avg_drone_lin_vel[X]', 'avg_drone_lin_vel[Y]', 'avg_drone_lin_vel[Z]',

        ]


        directory_csv = '/home/cnchano/vel_calc_data.csv' #os.path.join(os.getcwd(), 'src/drone_control/vel_calc_data2.csv')
        # self.get_logger().info(f'Getting csv file path: {directory_csv}...')
        if (self.iter_count == 0.0): #self.prev_ctrl_loop_timestamp is initialized at zero in the initialization function, therefore the first run of the script should have this value equal to zero
        # self.get_logger().info(f'creating Drone Conductor csv file...')
            with open(directory_csv, 'w', newline='') as f_object:
                writer_object = writer(f_object)
                writer_object.writerow(csv_list_header)
                writer_object.writerow(csv_list)
                # Close the file object
                f_object.close()
                self.iter_count += 1
        #    self.get_logger().info(f'Finished first write to csv file...')

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
        #////////////////////////////////////////////////////////////
        if (delta_time != 0.0):
            self.past_timestamp = timestamp
            self.past_drone_pos_if = drone_pos_if
        
    def call_gps_service(self):

        self.gps_req.drone_name = 'drone1'

        self.gps_future = self.gps_client.call_async(self.gps_req)
        rclpy.spin_until_future_complete(self.gps_sub_node, self.gps_future)
        return self.gps_future.result()



    def call_sim_clock_service(self):

        self.sim_clock_future = self.sim_clock_client.call_async(self.sim_clock_req)
        rclpy.spin_until_future_complete(self.sim_clock_sub_node, self.sim_clock_future)
        return self.sim_clock_future.result()
    



def main(args=None):
    rclpy.init(args=args)

    velocity_serv = Velocity_Serv()

    rclpy.spin(velocity_serv)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    velocity_serv.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()