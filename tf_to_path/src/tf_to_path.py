#!/usr/bin/env python

import rospy
import tf
import rotations 

import std_msgs
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Pose
import numpy as np

path_pub = None
path = Path()
    
rospy.init_node('tf_to_path', anonymous=True)
src_frame = None
dst_frame = None
try:
    src_frame = rospy.get_param("~src_frame")
    dst_frame = rospy.get_param("~dst_frame")  
except Exception as  e:
    print("could not get parameters")
    exit(1)    

path_pub = rospy.Publisher('/odom_path', Path, queue_size=1000)
rate = rospy.Rate(100.0)
listener = tf.TransformListener()


camToBase = None
while not rospy.is_shutdown():
    camera_trans = None
    camera_rot = None 
    try:
        (camera_trans,camera_rot) = listener.lookupTransform(src_frame, dst_frame, rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        continue
    camToBasePose = rotations.getPose(camera_trans,camera_rot)
    camToBase = rotations.homogeneous(camToBasePose)
    camToBase = np.linalg.inv(camToBase)
    break

while not rospy.is_shutdown():
    try:
        (trans,rot) = listener.lookupTransform(src_frame, dst_frame, rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        continue
    
    time =  rospy.Time.now()
    
    odomPose = rotations.getPose(trans,rot)
    odomMat = rotations.homogeneous(odomPose)
    cameraMat = np.dot(camToBase,odomMat)
    
    
    path.header=std_msgs.msg.Header()
    path.header.frame_id='odom'
    path.header.stamp= time
    
    poseStamped = PoseStamped()
    poseStamped.header=std_msgs.msg.Header()
    poseStamped.header.frame_id='odom'
    poseStamped.header.stamp= time
    
    poseStamped.pose = rotations.poseFromHomo(cameraMat)
      
    path.poses.append(poseStamped)    
    path_pub.publish(path)
    
    rate.sleep()