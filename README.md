# norlab_imu_tools

imu_attitude_to_tf node has to acknowledge Hector_slam: http://wiki.ros.org/hector_slam, BSD licence:
// Copyright (c) 2012, Stefan Kohlbrecher, TU Darmstadt
// All rights reserved.

## Parameters of the imu (and gps) to tf tool

**IMPORTANT** Skidoo moved backwards, so the GPS heading correction acts backwards as well. If used for anything else, find and disable the 180 deg rotation in the code...!

```xml
<remap from="imu_topic" to="/mti/sensor/imu" />   # Raw imu input topic 
<remap from="gps_odom_topic" to="/odom_utm" />    # UTM GPS coordinates input topic
<param name="odom_frame" value="world" />         # Fixed odometry frame

# Base link frame == IMU frame, orientation specified by the imu_alignment_rpy rosparam.
<param name="base_frame" value="imu_link" />      

# If false, translation=0. If true, translation from GPS UTM, set to zero when launched
<param name="publish_gps_translation" value="False" /> 

# Correction of the drifting heading by the information from GPS. 
<param name="apply_gps_heading_correction" value="True" /> # Apply or not
<param name="gps_heading_correction_weight" value="0.1" /> # Filtering factor (1==full correction from GPS applied at each step. 0.1 recommended for smooth behavior)
<param name="gps_heading_min_dist" value="1.5" /> # How long a GPS path segment should be for computing its tangent

# Fixed rotation between the physical IMU and the base_link frame
# Note that we neglect centrifugal acceleration, in skidoo, translation between IMU and Base link == 0
<rosparam param="imu_alignment_rpy">[3.14159265359, 0.0, 3.14159265359]</rosparam>  <!--Pelicase box for skidoo, all angles in radians! -->

# Initial magnetic north correction angle. Note that this value is later precised by the GPS information
<param name="mag_north_correction_yaw" value="0.27331833" /> <!-- 15.66 deg == 15°40'  => 0.27331833               0.43534373-->
```

## Interactive Static Transform tool

This tool helps finding a transformation between two TF frames by providing
an RVIZ interactive marker. The usage is straightforward:

* Set your parent and child frame names in the launch file
* Run the lauch file and also open the RVIZ
* The initial transformation is identity, but you can modify it by displaying
the interactive marker and moving it around
* The marker also offers a context menu, the only command which is there
prints the current transform value in the form of the *static_transform_publisher*
rosrun command