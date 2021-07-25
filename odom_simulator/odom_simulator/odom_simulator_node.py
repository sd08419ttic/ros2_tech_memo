#!/usr/bin/env python
# -*- coding: utf-8 -*-

import pandas as pd
# import for ros function
import rclpy
from rclpy.node import Node
import tf2_py
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Header
from nav_msgs.msg import Path, Odometry
from odom_simulator_func import OdomSimlatorFunc
from tf2_ros.transform_broadcaster import TransformBroadcaster

########################
# Simple Path follower #
########################
class OdomSimulatorNode(Node):

    ##################
    # Initialization #
    ##################
    def __init__(self):
        super().__init__('Odom_Simulator')

        #initialize publisher
        self.OdometryPub_ = self.create_publisher(Odometry, '/odom', 10)
        self._tf_Odompublisher = TransformBroadcaster(self)

        #initialize subscriber
        self.CmdvelSub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.sub_cmdvel_callback,
            10)

        self.CmdvelSub  # prevent unused variable warning

        #timer configuration
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        #self.path_follow_func = Path_follow_func()
        self.cnt = 0

        self.odomSimFunc = OdomSimlatorFunc()

    def timer_callback(self):
        # publish odom data
        #cmdvel = self.pathfollowerFunc.update_cmd_vel()
        #odom = Odometry()
        odom, tf_odom = self.odomSimFunc.update_odom()
        #calc cmdvel
        odom.header.stamp = self.get_clock().now().to_msg()
        self.OdometryPub_.publish(odom)
        tf_odom.header.stamp = self.get_clock().now().to_msg()
        self._tf_Odompublisher.sendTransform(tf_odom)
        self.cnt += 1
        if self.cnt %10 ==0:
            self.get_logger().info('Publishing Count: "%s"' % self.cnt)

    #subscriber call back for Odometry
    def sub_cmdvel_callback(self, msg):
        self.odomSimFunc.set_cmdvel_from_subscriber(msg)
        test = Twist()
        self.get_logger().info('I heard: "%s"' % msg.linear)

def main(args=None):
    rclpy.init(args=args)

    odomSimNode = OdomSimulatorNode()
    rclpy.spin(odomSimNode)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    odomSimNode.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

