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
initialPoseMat = None
baseToCamMat = None

def addPoseToPath(pose, stamp):
    global path, path_pub
    path.header=std_msgs.msg.Header()
    path.header.frame_id = 'odom'
    path.header.stamp = stamp
    
    poseStamped = PoseStamped()
    poseStamped.header=std_msgs.msg.Header()
    poseStamped.header.frame_id='odom'
    poseStamped.header.stamp= stamp

    poseStamped.pose = pose
    path.poses.append(poseStamped)    

def gtCallback(gtOdom):
    global initialPoseMat
    pose = gtOdom.pose.pose
    if initialPoseMat is None:
        initialPoseMat = rotations.homogeneous(pose)
        initialPoseMat = np.dot(initialPoseMat,baseToCamMat)
        initialPoseMat = np.linalg.inv(initialPoseMat)
    
    baseLinkMat = rotations.homogeneous(pose)
    camMat = np.dot(baseLinkMat,baseToCamMat)
    M = np.dot(initialPoseMat,camMat)
    
    pose = rotations.poseFromHomo(M)
    addPoseToPath(pose, gtOdom.header.stamp)
    
rospy.init_node('tf_to_path', anonymous=True)
src_frame = None
dst_frame = None
try:
    base_frame = rospy.get_param("~base_frame")
    cam_frame = rospy.get_param("~cam_frame") 
    gt_topic = rospy.get_param("~gt_topic") 
except Exception as  e:
    print("could not get parameters")
    exit(1)    

path_pub = rospy.Publisher('/gt_path', Path, queue_size=1000)
listener = tf.TransformListener()
rate = rospy.Rate(100)

while not rospy.is_shutdown():    
    try:
        (baseToCamTrans,baseToCamRot) = listener.lookupTransform(base_frame, cam_frame, rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        continue
    baseToCamPose = rotations.getPose(baseToCamTrans,baseToCamRot)  
    baseToCamMat = rotations.homogeneous(baseToCamPose)    
    break

rospy.Subscriber(gt_topic, Odometry, gtCallback)
#rospy.spin()

while not rospy.is_shutdown():
    #we have at least one measurment
    if initialPoseMat is not None:
        path_pub.publish(path)
    rate.sleep()
