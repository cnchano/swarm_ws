

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
from ptcloud_func_tools import pointcloud2_to_xyz_array

class Front_Pt_Cloud_Subscriber(Node):

    def __init__(self):
        super().__init__('front_pt_cloud_subscriber')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/sensor_camera_front/points',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        
        self.xyz_data = pointcloud2_to_xyz_array(msg.data, remove_nans=True)
        # self.get_logger().info('Timestamp: "%s"' % msg.header.stamp)
        #self.get_logger().info(f'Header: {msg.header}')
        #self.get_logger().info(f'Height: {msg.height}') #uint32/ height of point cloud
        #self.get_logger().info(f'Width: {msg.width}') #uint32/ width of point cloud
        #self.get_logger().info(f'Data: {msg.data}') #uint8/ point cloud data
        #self.get_logger().info(f'Point Step: {msg.point_step}') #uint32/ height of point cloud
        #self.get_logger().info(f'Width: {msg.row_step}') #uint32/ width of point cloud
        self.get_logger().info(f'XYZ Data: {self.xyz_data}')

def main(args=None):
    rclpy.init(args=args)

    front_pt_cloud_subscriber = Front_Pt_Cloud_Subscriber()

    rclpy.spin(front_pt_cloud_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    front_pt_cloud_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()