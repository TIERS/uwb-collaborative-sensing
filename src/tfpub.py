#!/usr/bin/env python  
import roslib
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

from sensor_msgs.msg import PointCloud2 as pc2

import rospy
import tf

import cv2
import threading
import numpy as np

br = tf.TransformBroadcaster()  
ugv_pose = [0,0,0,0,0,0,1]              #[x,y,z, x,y,z,w]
trans_offset = [0, 0, 0, 0, 0, 0]       #[x,y,z, roll, pitch, yaw]

# Dynamic parm adjustment 
def callback(object):
    pass

def parm_adjust_th(): 

    cv2.namedWindow('Pose_Adjuster')
    cv2.createTrackbar('pos_x', 'Pose_Adjuster', 205, 200 * 2, callback)
    cv2.createTrackbar('pos_y', 'Pose_Adjuster', 200, 200 * 2, callback)
    cv2.createTrackbar('pos_z', 'Pose_Adjuster', 200, 200 * 2, callback)
    cv2.createTrackbar('roll',  'Pose_Adjuster', 7540, 31415 * 2, callback)
    cv2.createTrackbar('pitch', 'Pose_Adjuster', 32015, 31415 * 2, callback)
    cv2.createTrackbar('yaw',   'Pose_Adjuster', 21659, 31415 * 2, callback)
    img = np.zeros((100, 400, 3), np.uint8)

    while(True):
        pos_x = (cv2.getTrackbarPos('pos_x', 'Pose_Adjuster') - 200)/10.0
        pos_y = (cv2.getTrackbarPos('pos_y', 'Pose_Adjuster') - 200)/10.0
        pos_z = (cv2.getTrackbarPos('pos_z', 'Pose_Adjuster') - 200)/10.0
        roll =  (cv2.getTrackbarPos('roll',  'Pose_Adjuster') - 31415)/10000.0
        pitch = (cv2.getTrackbarPos('pitch', 'Pose_Adjuster') - 31415)/10000.0
        yaw  =  (cv2.getTrackbarPos('yaw', 'Pose_Adjuster')   - 31415)/10000.0 

        img[:] = [ int(np.abs(pos_x+roll)) * 10 % 255,  \
                int(np.abs(pos_y+pitch))* 10 % 255,  \
                int(np.abs(pos_z+yaw))  * 10 % 255   ]
        if cv2.waitKey(10) & 0xFF == ord('q'):
            break
        
        global trans_offset
        trans_offset = [pos_x, pos_y, pos_z, roll, pitch, yaw ] 
        cv2.imshow('Pose_Adjuster', img)
    cv2.destroyAllWindows()  

def handle_uav_vio_pose(msg):
    # print("UAV VIO")
    pass

def handle_uav_uwb_pose(msg):  
    quaternion = (msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
    roll, pitch, yaw = tf.transformations.euler_from_quaternion(quaternion)
    
    global trans_offset # trans_offset = [pos_x, pos_y, pos_z, roll, pitch, yaw ] 

    quat = tf.transformations.quaternion_from_euler (roll + trans_offset[3], pitch + trans_offset[4],  yaw + trans_offset[5])
    br.sendTransform((msg.pose.position.x + trans_offset[0] , msg.pose.position.y + trans_offset[1], msg.pose.position.z + trans_offset[2]),
                    (quat[0], quat[1], quat[2], quat[3]),
                    rospy.Time.now(),
                    "/uav_pose",
                    "/uwb") 

def handle_ugv_uwb_pose(msg): 
    # Update UGV pose - Position from UWB
    global ugv_pose
    ugv_pose[0] = msg.position.x
    ugv_pose[1] = msg.position.y
    ugv_pose[2] = msg.position.z 

def handle_ugv_vio_pose(msg):
    # Update ugv pose - Orientation from UWB
    global ugv_pose
    ugv_pose[3] = msg.pose.pose.orientation.x
    ugv_pose[4] = msg.pose.pose.orientation.y
    ugv_pose[5] = msg.pose.pose.orientation.z
    ugv_pose[6] = msg.pose.pose.orientation.w
    # msg.pose.orientation.x
    br.sendTransform((ugv_pose[0], ugv_pose[1], ugv_pose[2]), 
                    (ugv_pose[3], ugv_pose[4], ugv_pose[5], ugv_pose[6]), 
                    rospy.Time.now(), 
                    "/ugv_pose", 
                    "/uwb") 

def handle_ugv_pc2(msg):
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "/ugv_lidar_pose"
    pc2_pub.publish(msg) 


if __name__ == '__main__':
    rospy.init_node('my_tf_broadcaster')

    # Receive UAV VIO pose
    rospy.Subscriber("/camera/odom/sample",
                     Odometry,
                     handle_uav_vio_pose
                     )

    # Receive UAV uwb pose + VIO orientation
    rospy.Subscriber("uav/mavros/vision_pose/pose",
                     PoseStamped,
                     handle_uav_uwb_pose
                     )

    # Receive ugv  uwb pose
    rospy.Subscriber("/dwm1001/tag/ugv/position",
                     Pose,
                     handle_ugv_uwb_pose
                     )

    # Receive UGV camera odometry pose 
    rospy.Subscriber("/camera/odom/sample",
                     Odometry,
                     handle_ugv_vio_pose
                     )

    # Receive UGV lidar PointCloud2
    rospy.Subscriber("/livox/lidar",
                     pc2,
                     handle_ugv_pc2
                     )

    # Start parm adjuster
    t1 = threading.Thread(target = parm_adjust_th)
    t1.setDaemon(True)
    t1.start()



    pc2_pub = rospy.Publisher('livox/lidar_pc2', pc2, queue_size=10)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():  
        br.sendTransform((0, 0, 0.5 ),
                        (0.0, 0.0, 0.0, 1.0),
                        rospy.Time.now(),
                        "/ugv_lidar_pose",
                        "/ugv_pose") 

        br.sendTransform((0, 0, -0.15),
                        (0.0, 0.0, 0.0, 1.0),
                        rospy.Time.now(),
                        "/d435_depth_optical_frame",
                        "/uav_pose") 

        rate.sleep()
    rospy.spin()