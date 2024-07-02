import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from drone_common.srv import SimClock
from std_msgs.msg import String
import numpy as np
from .constants import (
    DRONE_CONSTANT,
    FILTER_CONSTANT
)

from drone_common.srv import IMU

# Import writer class from csv module
from csv import writer
import os

class IMU_Subscriber(Node): #NOTES TO SELF: self.### will allow access of variable anywhere within the class

    def __init__(self):
        super().__init__('imu_subscriber')
        #Subscriber for main drone body
        self.subscription = self.create_subscription(
            Imu,
            'imu_main_body/out',
            self.imu_drone_sub_callback,
            10)
        #Subscriber for propeller FR1
        self.subscription = self.create_subscription(
            Imu,
            'imu_prop_FR1/out',
            self.imu_propFR1_sub_callback,
            10)
        #Subscriber for propeller FL2
        self.subscription = self.create_subscription(
            Imu,
            'imu_prop_FL2/out',
            self.imu_propFL2_sub_callback,
            10)
        #Subscriber for propeller RL3
        self.subscription = self.create_subscription(
            Imu,
            'imu_prop_RL3/out',
            self.imu_propRL3_sub_callback,
            10)

        #Subscriber for propeller RR4
        self.subscription = self.create_subscription(
            Imu,
            'imu_prop_RR4/out',
            self.imu_propRR4_sub_callback,
            10)
        self.subscription  # prevent unused variable warning (ROS2 comment)
        #Service to provide propeller angular velocities, drone angular velocities, drone linear velocity
        self.srv = self.create_service(IMU, 'IMU_data_service', self.IMU_data_service_callback)

        #//////////////////////////////////////////////
        self.sim_clock_sub_node = rclpy.create_node('imu_sim_clock_sub_node') #create sub-node for gps client
        #Initialize sim_clock Client
        self.sim_clock_client = self.sim_clock_sub_node.create_client(SimClock, 'sim_clock_service')
        while not self.sim_clock_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.sim_clock_req = SimClock.Request()
        #///////////////////////////////////////////////////////

        # create example publisher that outputs every second
        # self.publisher_ = self.create_publisher(String, 'zaidtopic', 10) #creates new publisher outputting string
        # self.timer = self.create_timer(1, self.zaid) #creates access to a new function, executed every 1 second

        #initiate variables and arrays for keeping data 
        #initialize starting linear velocity as zero
        self.get_logger().info(f'IMU variables initialized to zero')
        self.imu_drone_timestamp = 0.0
        self.drone_orient = np.array([[0.0],[0.0],[0.0],[0.0]]) #NOTE np.zeros gives a type error when compiling
        self.drone_ang_vel = np.array([[0.0],[0.0],[0.0]])
        self.drone_lin_accel = np.array([[0.0],[0.0],[0.0]])
        self.drone_lin_vel = np.array([[0.0],[0.0],[0.0]])
        self.past_drone_lin_vel = np.array([[0.0],[0.0],[0.0]])
        self.gravity = 9.807 #m/s^2

        self.imu_propFR1_timestamp = 0.0
        self.prop_ang_vel_FR1 = 0.0
        self.imu_propFL2_timestamp = 0.0
        self.prop_ang_vel_FL2 = 0.0
        self.imu_propRL3_timestamp = 0.0
        self.prop_ang_vel_RL3 = 0.0
        self.imu_propRR4_timestamp = 0.0
        self.prop_ang_vel_RR4 = 0.0

        # self.get_logger().info(f'Initialized Time Stamp: {self.imu_drone_timestamp}')
        # self.get_logger().info(f'Initialized Quarternion (orientation): {self.drone_orient}')
        # self.get_logger().info(f'Initialized Angular Velocity:  {self.drone_ang_vel}')
        # self.get_logger().info(f'Initialized Linear Accel: {self.drone_lin_accel}')
        # self.get_logger().info(f'Initialized Linear Vel: {self.past_drone_lin_vel}')

        #Initialize iteration counter for csv write to initialize file, then later add variables
        self.iter_count = 0.0

        self.AVG_WINDOW_SIZE = FILTER_CONSTANT['imu_accel_moving_avg_window']
        self.drone_lin_accel_matrix = np.array([[0.0],[0.0],[0.0]])
    #/////
    def IMU_data_service_callback(self, request, response):
        #request - determine how to request info for each individual drone when spawning multiple

        #Propeller rotational velocity data for service request 
        response.prop_ang_vel_fr1 = float(self.prop_ang_vel_FR1)
        response.imu_prop_fr1_timestamp = float(self.imu_propFR1_timestamp)
        response.prop_ang_vel_fl2 = float(self.prop_ang_vel_FL2)
        response.imu_prop_fl2_timestamp = float(self.imu_propFL2_timestamp)
        response.prop_ang_vel_rl3 = float(self.prop_ang_vel_RL3)
        response.imu_prop_rl3_timestamp = float(self.imu_propRL3_timestamp)
        response.prop_ang_vel_rr4 = float(self.prop_ang_vel_RR4)
        response.imu_prop_rr4_timestamp = float(self.imu_propRR4_timestamp)
        
        #Drone body angular velocity, linear acceleration and orientation
        response.drone_orient_x = float(self.drone_orient[0])
        response.drone_orient_y = float(self.drone_orient[1])
        response.drone_orient_z = float(self.drone_orient[2])
        response.drone_orient_w = float(self.drone_orient[3])

        response.drone_ang_vel_x = float(self.drone_ang_vel[0])
        response.drone_ang_vel_y = float(self.drone_ang_vel[1])
        response.drone_ang_vel_z = float(self.drone_ang_vel[2])

        response.drone_lin_accel_x = float(self.drone_lin_accel[0])
        response.drone_lin_accel_y = float(self.drone_lin_accel[1])
        response.drone_lin_accel_z = float(self.drone_lin_accel[2])

        response.imu_drone_timestamp = float(self.imu_drone_timestamp)

        response.drone_lin_vel_x = float(self.drone_lin_vel[0])
        response.drone_lin_vel_y = float(self.drone_lin_vel[1])
        response.drone_lin_vel_z = float(self.drone_lin_vel[2])
        
        return response



    def call_sim_clock_service(self):

        self.sim_clock_future = self.sim_clock_client.call_async(self.sim_clock_req)
        rclpy.spin_until_future_complete(self.sim_clock_sub_node, self.sim_clock_future)
        return self.sim_clock_future.result()

    def imu_propFR1_sub_callback(self, msg):
        #self.get_logger().info(f'Angular Velocity: x - {msg.angular_velocity.x}, y - {msg.angular_velocity.y}, z - {msg.angular_velocity.z}') #rad/sec
        
        #Pass previous timestamp,and angular velocity
        self.past_imu_drone_timestamp = self.imu_propFR1_timestamp
        self.past_prop_ang_vel_FR1 = self.prop_ang_vel_FR1
        #Pass current timestamp,and angular velocity
        self.prop_ang_vel_FR1 = msg.angular_velocity.z #rad/sec
        self.imu_propFR1_timestamp = (msg.header.stamp.sec)+(msg.header.stamp.nanosec*10**(-9)) #sec
        
    def imu_propFL2_sub_callback(self, msg):
        #self.get_logger().info(f'Angular Velocity: x - {msg.angular_velocity.x}, y - {msg.angular_velocity.y}, z - {msg.angular_velocity.z}') #rad/sec
        
        #Pass previous timestamp,and angular velocity
        self.past_imu_drone_timestamp = self.imu_propFL2_timestamp
        self.past_prop_ang_vel_FL2 = self.prop_ang_vel_FL2
        #Pass current timestamp,and angular velocity
        self.prop_ang_vel_FL2 = msg.angular_velocity.z #rad/sec
        self.imu_propFL2_timestamp = (msg.header.stamp.sec)+(msg.header.stamp.nanosec*10**(-9)) #sec #sec

    def imu_propRL3_sub_callback(self, msg):
        #self.get_logger().info(f'Angular Velocity: x - {msg.angular_velocity.x}, y - {msg.angular_velocity.y}, z - {msg.angular_velocity.z}') #rad/sec
        
        #Pass previous timestamp,and angular velocity
        self.past_imu_drone_timestamp = self.imu_propRL3_timestamp
        self.past_prop_ang_vel_RL3 = self.prop_ang_vel_RL3
        #Pass current timestamp,and angular velocity
        self.prop_ang_vel_RL3 = msg.angular_velocity.z #rad/sec
        self.imu_propRL3_timestamp = (msg.header.stamp.sec)+(msg.header.stamp.nanosec*10**(-9)) #sec

    def imu_propRR4_sub_callback(self, msg):
        #self.get_logger().info(f'Angular Velocity: x - {msg.angular_velocity.x}, y - {msg.angular_velocity.y}, z - {msg.angular_velocity.z}') #rad/sec
        
        #Pass previous timestamp,and angular velocity
        self.past_imu_drone_timestamp = self.imu_propRR4_timestamp
        self.past_prop_ang_vel_RR4 = self.prop_ang_vel_RR4
        #Pass current timestamp,and angular velocity
        self.prop_ang_vel_RR4 = msg.angular_velocity.z #rad/sec
        self.imu_propRR4_timestamp = (msg.header.stamp.sec)+(msg.header.stamp.nanosec*10**(-9)) #sec
    #////


        
    def imu_drone_sub_callback(self, msg):

        #Save timestamps with data to calculate derivatives and integrals of accel and velocity.
        #Save a moving average of IMU data to smooth out results, then output average result for service request
        #///////////////////////////////////////////////////
        #Drone main body
        #initialize variables with zero on first iteration of callback
        # # if (self.imu_drone_timestamp == None):
        # #     self.get_logger().info(f'IMU variables initialized to zero')
        # #     self.imu_drone_timestamp = 0
        # #     self.drone_orient = np.zeros(3,1)
        # #     self.drone_ang_vel = np.zeros(3,1)
        # #     self.drone_lin_accel = np.zeros(3,1)
        # #     self.past_drone_lin_vel = np.zeros(3,1)
            
        # #     self.get_logger().info(f'Initialized Time Stamp: {self.imu_drone_timestamp}')
        # #     self.get_logger().info(f'Initialized Quarternion (orientation): {self.drone_orient}')
        # #     self.get_logger().info(f'Initialized Angular Velocity:  {self.drone_ang_vel}')
        # #     self.get_logger().info(f'Initialized Linear Accel: {self.drone_lin_accel}')
        # #     self.get_logger().info(f'Initialized Linear Vel: {self.past_drone_lin_vel}')
        
        #Pass previous timestamp, orientation, angular velocity, and linear acceleration
        self.past_imu_drone_timestamp = self.imu_drone_timestamp
        self.past_drone_orient = self.drone_orient
        self.past_drone_ang_vel = self.drone_ang_vel
        self.past_drone_lin_accel = self.drone_lin_accel
        # self.get_logger().info(f'Past Time Stamp: {self.past_imu_drone_timestamp}')
        # self.get_logger().info(f'Past Quarternion (orientation): {self.past_drone_orient}')
        # self.get_logger().info(f'Past Angular Velocity:  {self.past_drone_ang_vel}')
        # self.get_logger().info(f'Past Linear Accel: {self.past_drone_lin_accel}')
        
        #/////////////////////////
        #NOTE: MOVING AVERAGE FOR LINEAR ACCELERATION
        (arr_col_size, arr_row_size) = self.drone_lin_accel_matrix.shape
        if (arr_row_size < self.AVG_WINDOW_SIZE):
            self.drone_lin_accel_matrix = np.append(self.drone_lin_accel_matrix, np.array([[msg.linear_acceleration.x],[msg.linear_acceleration.y],[msg.linear_acceleration.z ]]), axis=1)
            
        else:
            self.drone_lin_accel_matrix = np.append(self.drone_lin_accel_matrix, np.array([[msg.linear_acceleration.x],[msg.linear_acceleration.y],[msg.linear_acceleration.z ]]), axis=1)
            
            self.drone_lin_accel_matrix = np.delete(self.drone_lin_accel_matrix, 0, 1)
            
        kernel_array = np.ones(self.AVG_WINDOW_SIZE)/self.AVG_WINDOW_SIZE
        lin_accel_arr_conv_x = np.convolve(self.drone_lin_accel_matrix[0,:], kernel_array, mode='valid')
        lin_accel_arr_conv_y = np.convolve(self.drone_lin_accel_matrix[1,:], kernel_array, mode='valid')
        lin_accel_arr_conv_z = np.convolve(self.drone_lin_accel_matrix[2,:], kernel_array, mode='valid')
        
        self.drone_lin_accel = np.array([[lin_accel_arr_conv_x[-1]],[lin_accel_arr_conv_y[-1]],[lin_accel_arr_conv_z[-1]]])
        #/////////////////////////

        #update timestamp, orientation, angular velocity, and linear acceleration
        self.imu_drone_timestamp = (msg.header.stamp.sec)+(msg.header.stamp.nanosec*10**(-9))
        self.drone_orient = np.array([[float(msg.orientation.x)],[float(msg.orientation.y)],[float(msg.orientation.z)],[float(msg.orientation.w)]]) #quarternion
        self.drone_ang_vel = np.array([[msg.angular_velocity.x],[msg.angular_velocity.y],[msg.angular_velocity.z]]) #rad/sec
        
        #self.drone_lin_accel = np.array([[msg.linear_acceleration.x],[msg.linear_acceleration.y],[msg.linear_acceleration.z ]]) #m/s^2 - self.gravity NOTE: if using moving average, comment this line out
        
        # self.get_logger().info(f'drone_lin_accel: {self.drone_lin_accel}')
        # self.get_logger().info(f'drone_orient: {self.drone_orient}')

        # self.get_logger().info(f'IMU Drone Time Stamp: {self.imu_drone_timestamp}')
        # self.get_logger().info(f'Quarternion (orientation): {self.drone_orient}')
        # self.get_logger().info(f'Angular Velocity:  {self.drone_ang_vel}')
        # self.get_logger().info(f'Linear Accel: {self.drone_lin_accel}')
        # self.get_logger().info(f'Linear Vel: {self.past_drone_lin_vel}')
        #Calculate derivative of linear acceleration (linear jerk) NOT NECESSARY
        #self.drone_lin_jerk = np.divide((self.drone_lin_accel - self.past_drone_lin_accel), np.multiply(np.ones(3,1),(self.imu_drone_timestamp-self.past_imu_drone_timestamp)))
        
        #//////////////////////////////////////
        #Get time from node NOTE: DELETE LATER
        # ros_time = self.get_clock().now().to_msg() #rclpy.time.Time() #self.get_clock().now().to_msg()
        # self.get_logger().info(f'Getting ROS time now: {ros_time}')
        # self.ros_time_now = float(ros_time.sec) + float(ros_time.nanosec*10**(-9))
        # self.get_logger().info(f'Getting processed ROS time now: {self.ros_time_now}')

        #import timestamp
        # clock_data = self.call_sim_clock_service()
        #self.get_logger().info(f'GPS service data: {clock_data}')
        
        #Pass timestamp data
        # clock_timestamp = clock_data.sim_timestamp
        # self.get_logger().info(f'CLOCK timestamp: {clock_timestamp}')
        #///////////////////////////////////////////

        #Calculate linear velocity
        # self.get_logger().info(f'Previous Linear Vel: {self.past_drone_lin_vel}')
        self.drone_lin_vel = self.past_drone_lin_vel + np.multiply(np.divide((self.drone_lin_accel + self.past_drone_lin_accel),2), (self.imu_drone_timestamp-self.past_imu_drone_timestamp))
        self.past_drone_lin_vel = self.drone_lin_vel
        # self.get_logger().info(f'New Linear Vel: {self.drone_lin_vel}')
        # self.get_logger().info(f'New Past Linear Vel: {self.past_drone_lin_vel}')
        # self.get_logger().info(f'Linear Accel: x - {msg.linear_acceleration.x}, y - {msg.linear_acceleration.y}, z - {msg.linear_acceleration.z}')
        # self.get_logger().info(f'Angular Velocity: x - {msg.angular_velocity.x}, y - {msg.angular_velocity.y}, z - {msg.angular_velocity.z}')
        # self.get_logger().info(f'Quarternion (orientation): x - {msg.orientation.x}, y - {msg.orientation.y}, z - {msg.orientation.z}, z - {msg.orientation.w}')
        # self.get_logger().info('Time Stamp: "%s"' % msg.header.stamp.sec)
        # self.get_logger().info('Orientation: "%s"' % msg.orientation)
        # self.get_logger().info('Angular Velocity: "%s"' % msg.angular_velocity)
        # self.get_logger().info('Linear Accel: "%s"' % msg.linear_acceleration)
        
        #define variables to be used as feedback inputs into the controller
        # msg_pub = String() # Whatever type you are planning on using
        
        # msg.linear_accel = np.array([[msg.linear_acceleration.x],[msg.linear_acceleration.y],[msg.linear_acceleration.z]])
        # msg.angular_vel = np.array([[msg.angular_velocity.x],[msg.angular_velocity.y],[msg.angular_velocity.z]])
        # msg.orientation_quart = np.array([[msg.orientation.x],[msg.orientation.y],[msg.orientation.z],[msg.orientation.w]])
        # msg.timestamp = msg.header.stamp.sec
        # msg_pub.data = f'Hello acc {msg.linear_acceleration.x}'
        
        # self.publisher_.publish(msg_pub)
        
        
        #convert quarternion orientation to euler angles
        drone_orient_euler = np.zeros([3,1])
        drone_orient_euler = self.euler_from_quaternion(self.drone_orient[0], self.drone_orient[1], self.drone_orient[2], self.drone_orient[3])
        #//////////////////////////////////////////////////////////////
        #NOTE: WRITING TO CSV LIST IS REQUIRED TO PROPERLY CONTROL THE DRONE SINCE IT EFFECTS TIMING OF THE SYSTEM
        #//////////////////////////////////////////////////////////////
        # List that we want to add as a new row for data collection
        csv_list = [ 
        self.imu_drone_timestamp, self.past_imu_drone_timestamp, 
        self.drone_lin_accel[0], self.drone_lin_accel[1], self.drone_lin_accel[2], 
        self.past_drone_lin_accel[0], self.past_drone_lin_accel[1], self.past_drone_lin_accel[2],
        self.drone_lin_vel[0], self.drone_lin_vel[1], self.drone_lin_vel[2],
        self.drone_ang_vel[0], self.drone_ang_vel[1], self.drone_ang_vel[2],
        self.drone_orient[0], self.drone_orient[1], self.drone_orient[2], self.drone_orient[3],
        drone_orient_euler[0], drone_orient_euler[1], drone_orient_euler[2]
        ]
        csv_list_header = [
        'imu_drone_timestamp', 'past_imu_drone_timestamp', 
        'drone_lin_accel[x]', 'drone_lin_accel[y]', 'drone_lin_accel[z]', 
        'past_drone_lin_accel[x]', 'past_drone_lin_accel[y]', 'past_drone_lin_accel[z]',
        'drone_lin_vel[x]', 'drone_lin_vel[y]', 'drone_lin_vel[z]',
        'drone_ang_vel[x]', 'drone_ang_vel[y]', 'drone_ang_vel[z]',
        'drone_orient[x]', 'drone_orient[y]', 'drone_orient[z]', 'drone_orient[w]',
        'drone_orient_euler[x]', 'drone_orient_euler[y]', 'drone_orient_euler[z]'
        ]


        directory_csv = '/home/cnchano/imu_sub_data.csv' #os.path.join(os.getcwd(), 'src/drone_control/imu_sub_data2.csv')
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

def main(args=None):
    rclpy.init(args=args)

    imu_subscriber = IMU_Subscriber()

    rclpy.spin(imu_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    imu_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()