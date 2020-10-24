#!/usr/bin/env python

'''
    UGV Rotates to always point its Livox lidar to UAV.

    Positions based on UWB ranging and orientations based on VIO.
'''

__author__     = "Jorge Pena Queralta"
__version__    = "0.1"
__maintainer__ = "Jorge Pena Queralta"
__email__      = "jopequ@utu.fi"
__status__     = "Development"


import tf
import os
import sys
import math
import time
import rospy

from nav_msgs.msg       import Odometry

from geometry_msgs.msg  import Twist
from geometry_msgs.msg 	import PoseStamped

class UGV_Follower :

    def __init__(self) :

        """
        Initialize the node, open serial port
        """

        # Init node
        rospy.init_node('UGV_UAV_Follower', anonymous=False)

        # Get topics from params
        self.cmd_vel_topic = rospy.get_param('~cmd_vel_topic', "/cmd_vel")
        self.uav_uwb_topic = rospy.get_param('~uav_uwb_topic', "/dwm1001/tag/dronie/position")
        self.ugv_uwb_topic = rospy.get_param('~ugv_uwb_topic', "/dwm1001/tag/ugv/position")

        # Publish data
        self.speed_pub 	 = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=10)

        # Basic pose
        self.x = 0
        self.y = 0
        self.uav_x = 99
        self.uav_y = 0     # By default will start looking to the positive 'x' direction
        self.orientation_z = 0

    def run(self) :
        '''
            Creates the velocity publication timer
            Goes into infinite loop with rospy spin()
        '''

        self.move_timer = rospy.Timerrospy.Duration(0.05, self.update_vel)

        # Infinite loop
        rospy.spin()

    def update_vel(self) :
        '''
            TO DO
            Only for testing very basic twist calculation to converge in ~2 seconds.
            Min rotational speed is 0.5 rad/s
        '''
        objective_yaw = math.atan2(self.uav_y-self.y, self.uav_x-self.x)

        twist = Twist()

        # Do nothing if yaw error < 0.2
        yaw_diff = abs(self.yaw-objective_yaw)
        if yaw_diff < 0.5 :
            return

        # Increase yaw
        if self.yaw < objective_yaw and yaw_diff < math.pi / 2 :
            twist.angular.z = 0.5 + yaw_diff/4
        elif self.yaw > objective_yaw and yaw_diff > math.pi / 2 :
            twist.angular.z = 0.5 + yaw_diff/4
        # Decrease yaw
        elif self.yaw < objective_yaw and yaw_diff < math.pi / 2 :
            twist.angular.z = -0.5 - yaw_diff/4
        else :
            twist.angular.z = -0.5 - yaw_diff/4
        
        self.speed_pub(twist)

    def update_ugv_pos(self, pos) :
        self.x = pos.position.x
        self.y = pos.position.y

    def update_uav_pos(self, pos) :
        self.uav_x = pos.position.x
        self.uav_y = pos.position.y

    def vio_cb(self, odom) :
        '''
            Update orientation from VIO
        '''

        quaternion = (
            odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y,
            odom.pose.pose.orientation.z,
            odom.pose.pose.orientation.w)

        euler = tf.transformations.euler_from_quaternion(quaternion)

        self.yaw = euler[2]


        

