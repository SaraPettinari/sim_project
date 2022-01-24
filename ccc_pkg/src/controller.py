#!/usr/bin/env python3
import rclpy
import numpy as np
import math
import random
import os

from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
from std_srvs.srv import Empty


class Controller(Node):

    def __init__(self):
        super().__init__('controller')
        self.counter = 1
        self.velocity = Twist()
        self.isRotating = False
        self.roll = self.pitch = self.yaw = 0.0
        self.ang_vel = 0.5
        self.cases = ['MoveRandom', 'MovePrior', 'MoveEast', 'MoveWest']
        self.behavior_case = self.cases[random.randint(0, len(self.cases)-1)]
        self.prior_flag = True
        self.position_p = Odometry()

        self.laser_sub = self.create_subscription(
            LaserScan, 'laser_scan', self.laser_detail, 10)
        self.create_subscription(Odometry, 'odom', self.get_position, 10)

        self.velocity_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.velocity_callback)

        self.cli = self.create_client(Empty, '/reset_simulation')
        self.req = Empty.Request()

    # send client request to reset the simulation
    def reset_simulation(self):
        self.future = self.cli.call_async(self.req)
        os.system("killall -9 gazebo & killall -9 gzserver  & killall -9 gzclient")


    # publish on Velocity topic
    def velocity_callback(self):
        self.velocity_pub.publish(self.velocity)


    # Management of current robot's position and orientation

    def get_position(self, msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y,
                            orientation_q.z, orientation_q.w]
        # conversion to roll, pitch and yaw
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(orientation_list)

        self.position_p = msg.pose.pose.position
        x = self.position_p.x

        # check if the robot has reached the final goal
        if x > 9.5:
            self.get_logger().info(self.behavior_case + ' -> Arrivato a destinazione!')
            # stop its movement
            self.velocity.linear.x = 0.0
            self.velocity.angular.z = 0.0
            self.velocity_pub.publish(self.velocity)

            self.reset_simulation()  # reset the simulation


        # check if it is walking straight but with a yaw != 0
        if (self.yaw > 0.005 or self.yaw < -0.005) and self.velocity.angular.z == 0.0:
            if not self.isRotating:
                self.rotate(0)

    # Get laser sensor details
    def laser_detail(self, laser_info: LaserScan):
        ranges = laser_info.ranges

        range_split = np.array_split(ranges, 3)

        range_dx = range_split[0]
        range_c = range_split[1]
        range_sx = range_split[2]

        if self.behavior_case == 'MoveEast':
            # self.moveEAST(range_dx, range_c, range_sx)
            self.move(range_dx, range_c, range_sx, 'E')
        elif self.behavior_case == 'MoveWest':
            # self.moveWEST(range_dx, range_c, range_sx)
            self.move(range_dx, range_c, range_sx, 'W')
        elif self.behavior_case == 'MovePrior':
            self.movePRIOR(range_dx, range_c, range_sx)
        elif self.behavior_case == 'MoveRandom':
            self.move(range_dx, range_c, range_sx, 'R')

    # Rotation
    def rotate(self, angle):
        self.isRotating = True
        target_rad = angle*math.pi/180
        check = target_rad-self.yaw
        self.velocity.angular.z = self.ang_vel * (check)
        if check > 0 and check < 0.002:
            self.velocity.linear.x = 0.5
            self.isRotating = False

    def move(self, rdx, rc, rsx, where):
        rdx_split = np.array_split(rdx, 3)
        rdx_danger = rdx_split[2]  # set a warning zone - R

        rsx_split = np.array_split(rsx, 3)
        rsx_danger = rsx_split[0]  # set a warning zone - L

        check_val_dx = np.isfinite(rdx_danger)
        check_val_c = np.isfinite(rc)
        check_val_sx = np.isfinite(rsx_danger)

        angle_45 = -45
        angle_90 = 90
        angle_20 = 20

        if where == 'E':
            angle_45 = -angle_45
            angle_90 = -angle_90
            angle_20 = -angle_20
        elif where == 'R':
            angle_45 = angle_90 = angle_20 = random.randint(-90, 90)

        # if it is in front of a wall
        if True in check_val_sx and True in check_val_c and True in check_val_dx:
            # if it is totally blocked
            if all(i for i in (np.isfinite(rsx))) and all(i for i in (np.isfinite(rdx))) and all(i for i in check_val_c):
                self.rotate(0)
                self.velocity.linear.x = -0.4
            else:
                self.rotate(angle_45)  # 45 degrees turn left
        # if it is in the navigation area
        elif True in check_val_c:
            self.rotate(angle_90)
        elif True in check_val_dx:
            self.rotate(0)
        elif True in rsx_danger:
            self.rotate(angle_20)
        else:
            self.isRotating = False
            self.velocity.linear.x = 0.7
            self.velocity.angular.z = 0.0

    # Action definition: MOVE PRIOR
    def movePRIOR(self, rdx, rc, rsx):
        # not Flag --> Right
        if not self.prior_flag:
            self.move(rdx, rc, rsx, 'E')
            if not self.isRotating:
                self.prior_flag = True
        # Flag --> Left
        else:
            self.move(rdx, rc, rsx, 'W')
            if self.isRotating:
                self.prior_flag = False



def setup():
    rclpy.init()
    controller_node = Controller()
    rclpy.spin(controller_node)

    if KeyboardInterrupt:
        rclpy.shutdown()


if __name__ == '__main__':
    setup()