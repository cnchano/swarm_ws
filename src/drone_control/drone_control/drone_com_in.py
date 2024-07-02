import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from drone_common.msg import DroneIntercom
from drone_common.srv import DroneIntercomIn

import numpy as np
import re
from array import array

from .constants import (
    POS_CONTROL,
    DRONE_CONSTANT,
    BEHAVIORAL_CTRL,
    CTRL_DT
)

class DroneComIn(Node):

    def __init__(self):
        super().__init__('drone_com_in')
        
        #MULTITHREADING DECLARATION
        internal_com_in_service_cb_group =ReentrantCallbackGroup() # MutuallyExclusiveCallbackGroup() # ReentrantCallbackGroup()
        com_in_subscriber_cb_group = ReentrantCallbackGroup() # MutuallyExclusiveCallbackGroup() # ReentrantCallbackGroup()

        #CREATE SUBSCRIBER
        self.drone_intercom_subscription = self.create_subscription(
        DroneIntercom,
        '/intercom_msg', #create absolute naming path to not create multiple topics with namespace prefix
        self.drone_intercom_listener_callback,
        10, #que size for QoS. In case messages are recieved faster than they are processed
        callback_group=com_in_subscriber_cb_group,
        )
        self.drone_intercom_subscription  # prevent unused variable warning
        
        #PARAMETER DECLARATION
        self.declare_parameter('swarm_size')
        self.swarm_size = self.get_parameter('swarm_size').get_parameter_value().integer_value
        self.get_logger().info(f'drone swarm_size parameter: {self.swarm_size}')
        
        #DRONE MESSAGE DICTIONARY INTIALIZATION
        self.drone_dictionary = {}
        for i in range(self.swarm_size): #create a dictionary to hold the drone data of the number of drones specified
            count = i+1
            
            iter_drone_dictionary = {
                'timestamp_'+ f'{count}': 0.0,
                'velocity_'+ f'{count}': [0.0,0.0, 0.0],
                'position_'+ f'{count}': [0.0, 0.0, 0.0], #initialize as NONE to ensure no false data is used in behavioral controller of other drones
                }
            self.drone_dictionary.update(iter_drone_dictionary) #concatenate dictionary with drone data placeholder
            
        #CREATE SERVICE
        self.drone_com_srv = self.create_service(
            DroneIntercomIn, 
            'drone_com_in_service', 
            self.drone_com_in_callback, 
            callback_group=internal_com_in_service_cb_group
            )
    

    def drone_intercom_listener_callback(self, msg):
        
        drone_id = re.findall('\d',msg.drone_id) #look in the drone_id passed (also used for the namespace) and extract the drone id number
        drone_id = int(drone_id[0]) #convert the string value into an integer


        #Update the drone data to the appropriate drone_id 
        #drone_dictionary['drone_id_'+f'{drone_id}']
        self.drone_dictionary['timestamp_'+f'{drone_id}'] = msg.timestamp
        self.drone_dictionary['velocity_'+f'{drone_id}'] = msg.velocity
        self.drone_dictionary['position_'+f'{drone_id}'] = msg.position
    
        # self.get_logger().info(f'drone dictionary data from msg: {self.drone_dictionary}')
        # self.get_logger().info(f'drone dictionary data from msg: {self.drone_dictionary["position_"+f"{drone_id}"]}')

    def drone_com_in_callback(self, request, response):
        # Initialize empty arrays to store data
        timestamp_list = np.array([])
        velocity_list = np.array([])
        position_list = np.array([])
        # drone_dict_copy = {}

        # Iterate over the swarm size
        for i in range(self.swarm_size): 
            count = int(i+1)
            count_str = str(count)

            
            # self.get_logger().info(f'drone timestamps dictionary extraction [drone_1, drone_2, drone_3]: {np.array(self.drone_dictionary["timestamp_1"])}')
            # self.get_logger().info(f'drone timestamps dictionary extraction [drone_1, drone_2, drone_3]: {type(self.drone_dictionary["timestamp_1"])}')
            # self.get_logger().info(f'drone timestamps dictionary extraction [drone_1, drone_2, drone_3]: {str(self.drone_dictionary["timestamp_"+count_str])}')
            
            # Extract data from the drone_dictionary using the count
            timestamp_list = np.concatenate((timestamp_list,(self.drone_dictionary['timestamp_'+count_str])), axis = None) # timestamp_list.append((self.drone_dictionary['timestamp_'+count_str]))
            velocity_list = np.concatenate((velocity_list,(self.drone_dictionary['velocity_'+count_str])), axis = None)# velocity_list.append((self.drone_dictionary['velocity_'+count_str]))
            position_list = np.concatenate((position_list,(self.drone_dictionary['position_'+count_str])), axis = None) #position_list.append((self.drone_dictionary['position_'+count_str]))

        # self.get_logger().info(f'drone timestamps [drone_1, drone_2, drone_3]: {timestamp_list}')
        # self.get_logger().info(f'drone velocity [drone_1, drone_2, drone_3]: {velocity_list}')
        # self.get_logger().info(f'drone position [drone_1, drone_2, drone_3]: {position_list}')
        # self.get_logger().info(f'drone timestamps [drone_1, drone_2, drone_3]: {timestamp_list.tolist()}')
        # self.get_logger().info(f'drone velocity [drone_1, drone_2, drone_3]: {velocity_list.tolist()}')
        # self.get_logger().info(f'drone position [drone_1, drone_2, drone_3]: {position_list.tolist()}')
        
        # Assign the lists of data to the response object
        response.swarm_size = self.swarm_size
        response.timestamp = timestamp_list.tolist()
        response.velocity = velocity_list.tolist()
        response.position = position_list.tolist()

        return response
    

def main(args=None):
    rclpy.init(args=args)

    drone_com_in = DroneComIn()
    
    executor = MultiThreadedExecutor() #add multi-threading to the node
    executor.add_node(drone_com_in)


    # rclpy.spin(drone_com_in) #spin using executor

    try:
        drone_com_in.get_logger().info('Beginning client, shut down with CTRL-C')
        executor.spin()
    except KeyboardInterrupt:
        drone_com_in.get_logger().info('Keyboard interrupt, shutting down.\n')


    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    drone_com_in.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()