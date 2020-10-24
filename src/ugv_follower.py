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

    def update_ugv_pos(self, pos) :
        pass

    def update_uav_pos(self, pos) :
        pass

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


        

