import rclpy
from rclpy.node import Node

from rosgraph_msgs.msg import Clock
from drone_common.srv import SimClock 
from sensor_msgs.msg import Imu
class Clock_Subscriber(Node):

    def __init__(self):
        super().__init__('clock_subscriber')
        
        #Subscriber for main drone body
        self.subscription = self.create_subscription(
        Imu,
        'imu_main_body/out',
        self.imu_drone_sub_callback,
        10)
        

        #Service to provide GPS coordinates of drone in cartesian coordinates against a reference location
        self.srv = self.create_service(SimClock, 'sim_clock_service', self.sim_clock_callback)
        
        self.past_imu_drone_timestamp = 0.0
        self.timestamp = 0.0

    def sim_clock_callback(self, request, response):

        response.sim_timestamp = float(self.timestamp)

        return response

    def imu_drone_sub_callback(self, msg):

        #Pass previous timestamp, 
        self.past_imu_drone_timestamp = self.timestamp

        #update timestamp,
        self.timestamp = (msg.header.stamp.sec)+(msg.header.stamp.nanosec*10**(-9))

        #self.get_logger().info(f'CLOCK SUB Time Stamp: {self.timestamp}')



    # def clock_listener_callback(self, msg):
    #     #take clock message data
    #     self.timestamp = (msg.clock.sec)+(msg.clock.nanosec*10**(-9))
    #     #self.get_logger().info(f'clock_listener_callback... {self.timestamp}')
    



def main(args=None):
    rclpy.init(args=args)

    clock_subscriber = Clock_Subscriber()

    rclpy.spin(clock_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    clock_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()