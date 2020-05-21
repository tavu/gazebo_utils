# ros gazebo_utils
Utilities for Gazebo simulator.

## Packages: ## 
1. depth_noise
2. depth_rgb_sync

### depth_noise ###
#### Nodes ####
1. depth_noise
2. image_converter
2. gazebo_utils

#### depth_noise ####
Adds gaussian noise to depth.
#### Parameters ####
> in_topic Input depth
> std standard deviation 

#### Output topics ####
> /camera/depth/image_noise

#### image_converter ####
Convert rgb image to different type.
#### Parameters ####
> in_topic Input topic
> out_type Type of the output image

#### Output topics ####
> /depth0/image_raw Depth topic
> /cam0/image_raw Rgb topic


### depth_rgb_sync ###
Syncs rgb and depth topics.
#### Parameters ####
> depth_in_topic Depth topic.
> rgb_in_topic Rgb Topic.
> rate Output rate.

#### Output topics ####
> /camera/depth/image_noise

### tf_to_path ###
#### Nodes ####
1. tf_to_path
2. gt_to_path

#### tf_to_path ####
Publish a path message (nav_msgs/Path) of the trajectory of the camera according to odom.
#### Parameters ####
> src_frame Odometry frame
> dst_frame Camera frame

#### Output topics ####
/odom_path

#### gt_to_path ####
Publish a path message (nav_msgs/Path) of the trajectory of the camera according to ground truth.
#### Parameters ####
> src_frame Frame of the ground truth
> dst_frame Camera frame
> gt_topic Ground truth topic

#### Output topics ####
/gt_path


#### gazebo_utils ####
A package with the urdf of turtlebot with asus xtion.
These urdfs usually install into the /opt filesystem so this package is convenient if you wish to change these parameters locally.
Wordls files should be placed inside of 'worlds' directory.


## Run: ## 
'''
roslaunch gazebo_utils gazebo_turtlebot_xtion.launch

'''

## Record bag: ## 
'''
cd scripts
./rec_gazebo.sh <bag_name.bag>
'''
