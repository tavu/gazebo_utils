#!/usr/bin/env python

import rospy
import tf
import rotations 

import std_msgs
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseStamped, Pose
import numpy as np

path_pub = None
path = Path()
initialPoseMat = None
baseToCamMat = None

def tfCallback(tf_mess):
    
    firstTf = tf_mess.transforms[0]
    transform = firstTf.transform
    p = Pose()    
    p.position.x = transform.translation.x
    p.position.y = transform.translation.y
    p.position.z = transform.translation.z
    
    p.orientation.x = transform.rotation.x
    p.orientation.y = transform.rotation.y
    p.orientation.z = transform.rotation.z
    p.orientation.w = transform.rotation.w
    
    gtCallback(p, firstTf.header.stamp)

def addPoseToPath(pose, stamp):
    global path, path_pub
    path.header=std_msgs.msg.Header()
    path.header.frame_id = 'odom'
    path.header.stamp = stamp
    
    poseStamped = PoseStamped()
    poseStamped.header=std_msgs.msg.Header()
    poseStamped.header.frame_id='odom'
    poseStamped.header.stamp= stamp
    #poseStamped.header.stamp= rospy.Time(0)
    poseStamped.header.stamp= rospy.Time.now()
    

    poseStamped.pose = pose
    path.poses.append(poseStamped)    

def gtCallback(pose, stamp):
    global initialPoseMat
    if initialPoseMat is None:
        initialPoseMat = rotations.homogeneous(pose)
        initialPoseMat = np.dot(initialPoseMat,baseToCamMat)
        initialPoseMat = np.linalg.inv(initialPoseMat)
    
    baseLinkMat = rotations.homogeneous(pose)
    camMat = np.dot(baseLinkMat,baseToCamMat)
    M = np.dot(initialPoseMat,camMat)
    
    pose = rotations.poseFromHomo(M)
    addPoseToPath(pose, stamp)
    
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
    except Exception as e:
        print(e)
        continue
    baseToCamPose = rotations.getPose(baseToCamTrans,baseToCamRot)  
    baseToCamMat = rotations.homogeneous(baseToCamPose)    
    print(baseToCamMat)
    break

rospy.Subscriber(gt_topic, TFMessage, tfCallback)
#rospy.spin()

while not rospy.is_shutdown():
    #we have at least one measurment
    if initialPoseMat is not None:
        path_pub.publish(path)
    rate.sleep()
