import rclpy
from rclpy.node import Node


from drone_common.srv import BehaviorControl
from drone_common.srv import IMU
from drone_common.srv import GPStoENU
from drone_common.srv import SimClock
from drone_common.srv import VelCalc

from drone_common.msg import DroneIntercom

import numpy as np
from csv import writer
from array import array

from .constants import (
    POS_CONTROL,
    DRONE_CONSTANT,
    BEHAVIORAL_CTRL,
    CTRL_DT
)

class DroneComOut(Node):

    def __init__(self):
        super().__init__('drone_com_out')
        self.drone_intercom_publisher = self.create_publisher(DroneIntercom, '/intercom_msg', 10) 
        timer_period = CTRL_DT['Ts']  # seconds
        self.timer = self.create_timer(timer_period, self.drone_intercom_publisher_callback)
        
        
        self.declare_parameter('drone_ns')
        self.drone_namespace = self.get_parameter('drone_ns').get_parameter_value().string_value 
        # self.get_logger().info(f'drone namespace parameter: {self.drone_namespace}')
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
        #//////////////////////////////////


        #Initialize iteration counter for csv write to initialize file, then later add variables
        self.iter_count = 0.0
        
    def drone_intercom_publisher_callback(self):
        #self.get_logger().info('behavior_controller_calc...')
        
        # #import IMU data as client/ convert IMU data WRT inertial frame
        # imu_data = self.call_imu_service()
        # #self.get_logger().info(f'IMU service data: {imu_data}')
        
        #import GPS data as client/ convert IMU data WRT inertial frame
        gps_data = self.call_gps_service()
        #self.get_logger().info(f'GPS service data: {gps_data}')

        #import timestamp
        clock_data = self.call_sim_clock_service()
        #self.get_logger().info(f'GPS service data: {clock_data}')

        #import velocity
        velocity_data = self.call_vel_calc_service()
        
        #Pass timestamp data
        timestamp = clock_data.sim_timestamp

        #Pass gps data
        drone_pos_if = array("f",[gps_data.de, gps_data.dn, gps_data.du]) #along x-axis, y-axis, z-axis

        

        

        #////////////////////////////////////
        
        msg = DroneIntercom()
        msg.drone_id = str(self.drone_namespace) #drone namespace to have drone identification associated with data
        msg.timestamp = float(timestamp)
        msg.velocity = array("f", velocity_data.drone_vel_if)
        msg.position = drone_pos_if
        self.drone_intercom_publisher.publish(msg)
        
        # #////////////////////////////////////

        # # List that we want to add as a new row for data collection
        # csv_list = [
        # timestamp,

        # ]

        # csv_list_header = [
        # "timestamp",
        # ]

        # directory_csv = '/home/cnchano/behavior_ctrl_out.csv' #os.path.join(os.getcwd(), 'src/drone_control/behavior_ctrl_out.csv')
        # # self.get_logger().info(f'Getting csv file path: {directory_csv}...')
        # if (self.iter_count == 0.0): #self.prev_ctrl_loop_timestamp is initialized at zero in the initialization function, therefore the first run of the script should have this value equal to zero
        #     # self.get_logger().info(f'creating Behavior Ctrl csv file...')
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

    drone_com_out = DroneComOut()

    rclpy.spin(drone_com_out)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    drone_com_out.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()