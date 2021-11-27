#!/usr/bin/env python3

# Spawn an SDF or URDF file into Gazebo

import sys

import rclpy
import transformations
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose


def inject(xml: str, initial_pose: Pose, namespace: str):
    
    # Creation of a ROS node, to call the SpawnEntity service
    rclpy.init()
    node = rclpy.create_node('inject_node')
    client = node.create_client(SpawnEntity, 'spawn_entity')

    if not client.service_is_ready():
        node.get_logger().info('waiting for spawn_entity service...')
        client.wait_for_service()

    request = SpawnEntity.Request()
    request.xml = xml
    request.robot_namespace = namespace
    request.initial_pose = initial_pose
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        node.get_logger().info('response: %r' % future.result())
    else:
        raise RuntimeError('exception while calling service: %r' % future.exception())

    node.destroy_node()
    rclpy.shutdown()


if len(sys.argv) < 6:
    print('Missing some parameters...')
    sys.exit(1)

f = open(sys.argv[1], 'r')

# Robot initial position definition
p = Pose()
p.position.x = float(sys.argv[2])
p.position.y = float(sys.argv[3])
p.position.z = float(sys.argv[4])
q = transformations.quaternion_from_euler(0, 0, float(sys.argv[5]))
p.orientation.w = q[0]
p.orientation.x = q[1]
p.orientation.y = q[2]
p.orientation.z = q[3]

inject(f.read(), p, str(sys.argv[6]))