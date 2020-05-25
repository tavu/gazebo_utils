# ros gazebo_utils
Utilities for Gazebo simulator.
It provides various packages for convenience such as turtlebot's urdf with asus xtion, image/depth synchronization, node that adds gaussian noise to depth and 
also converts ground truth of the base to camer frame.
It also contains a pre-build world.

## Initialize ## 
1. copy/link main directory to catking workspace.
2. Compile (catkin_make)

### Download gazebo models ### 
Download extra models for gazebo from 
> http://data.nvision2.eecs.yorku.ca/3DGEMS/

Models should be downloaded from DOWNLOAD section of the above link (world is not needed).
All the models should be downloaded and its content should be placed in:
> ~/.gazebo/models

*Important*
After the extraction of the models, do not copy the main folder to "~/.gazebo/models", copy only the inside directories.



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
Input depth
> in_topic 

standard deviation 
> std 

#### Output topics ####
> /camera/depth/image_noise

#### image_converter ####
Convert rgb image to different type.
#### Parameters ####
Input topic
> in_topic 

Type of the output image
> out_type 

#### Output topics ####
Depth topic
> /depth0/image_raw 

Rgb topic
> /cam0/image_raw 


### depth_rgb_sync ###
Syncs rgb and depth topics.
#### Parameters ####
Depth topic.
> depth_in_topic 

Rgb Topic.
> rgb_in_topic

Output rate.
> rate 

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
> /odom_path

#### gt_to_path ####
Publish a path message (nav_msgs/Path) of the trajectory of the camera according to ground truth.
#### Parameters ####
Frame of robot's base
> base_frame 

Camera frame
> cam_frame 

Ground truth topic
> gt_topic 

#### Output topics ####
> /gt_path


#### gazebo_utils ####
A package with the urdf of turtlebot with asus xtion.
These urdfs usually install into the /opt filesystem so this package is convenient if you wish to change these parameters locally.
Wordls files should be placed inside of 'worlds' directory.


## Run ## 
> roslaunch gazebo_utils gazebo_turtlebot_xtion.launch

## Record bag: ## 
> cd scripts
> ./rec_gazebo.sh <bag_name.bag>


## cite ##
A.Rasouli, J.K. Tsotsos. "The Effect of Color Space Selection on Detectability and Discriminability of Colored Objects." arXiv preprint arXiv:1702.05421 (2017).
arXiv preprint PDF (3 MB)