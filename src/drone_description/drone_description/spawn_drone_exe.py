"""Script used to spawn a robot in a generic position."""

import argparse
import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SpawnEntity
import rclpy

import xacro


def main():
    # GET INPUT ARGUMENTS FROM USER
    
    parser = argparse.ArgumentParser(description='Spawn Robot into Gazebo')
    parser.add_argument('-drone_urdf', '--drone_urdf', type=str, default='-',
                        help='File path to the xacro file drone model')
    parser.add_argument('-drone_name', '--drone_name', type=str, default='-',
                        help='Name of the drone to spawn')
    parser.add_argument('-drone', '--drone_namespace', type=str, default='default_ns',
                        help='ROS2 namespace to apply to the tf and plugins')
    parser.add_argument('-namespace_active_bool', '--namespace_active_bool', type=bool, default=False,
                        help='Whether to disable namespacing')
    parser.add_argument('-drone_spawn_x', type=float, default=0,
                        help='the x component of the initial position [meters]')
    parser.add_argument('-drone_spawn_y', type=float, default=0,
                        help='the y component of the initial position [meters]')
    parser.add_argument('-drone_spawn_z', type=float, default=0,
                        help='the z component of the initial position [meters]')

    args, unknown = parser.parse_known_args()

    # START NODE
    rclpy.init()
    node = rclpy.create_node('entity_spawner')

    node.get_logger().info(
        'Creating Service client to connect to `/spawn_entity`')
    client = node.create_client(SpawnEntity, '/spawn_entity')

    node.get_logger().info('Connecting to `/spawn_entity` service...')
    if not client.service_is_ready():
        client.wait_for_service()
        node.get_logger().info('...connected!')

    #GET XML DRONE DESCRIPTION
    urdf_file_path = args.drone_urdf # Get Xacro (drone model) file path
    print(urdf_file_path)

    robot_description_config = xacro.process_file(urdf_file_path, mappings={'robot_name_arg': args.drone_name}) #remap link/joint namespaces in xacro file
    robot_desc = robot_description_config.toxml() #convert xacro to xml string
    

    # SET DATA REQUEST TO SPAWN_ENTITY SERVICE
    request = SpawnEntity.Request()
    request.name = args.drone_name
    request.xml = robot_desc
    request.initial_pose.position.x = float(args.drone_spawn_x)
    request.initial_pose.position.y = float(args.drone_spawn_y)
    request.initial_pose.position.z = float(args.drone_spawn_z)

    if args.namespace_active_bool is True:
        node.get_logger().info('spawning `{}` on namespace `{}` at {}, {}, {}'.format(
            args.drone_name, args.drone_namespace, args.drone_spawn_x, args.drone_spawn_y, args.drone_spawn_z))

        request.robot_namespace = args.drone_namespace
        print(args.drone_namespace)

    else:
        node.get_logger().info('spawning `{}` at {}, {}, {}'.format(
            args.drone_name, args.drone_spawn_x, args.drone_spawn_y, args.drone_spawn_z))

    node.get_logger().info('Spawning Robot using service: `/spawn_entity`')

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        print('response: %r' % future.result())
    else:
        raise RuntimeError(
            'exception while calling service: %r' % future.exception())

    node.get_logger().info('Done! Shutting down node.')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()