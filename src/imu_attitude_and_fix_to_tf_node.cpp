//=================================================================================================
// Copyright (c) 2012, Stefan Kohlbrecher, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================


// The original node modified by Vladimir Kubelka, kubelvla@gmail.com for the purpose of relaying IMU messages to TF (for visualization and mapping w/o odometry information)
// Licence: BSD
// Laval University, NORLAB, 2019

#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Quaternion.h"
#include <cmath>
#include <sstream>


// frame names
std::string p_odom_frame_;
std::string p_base_frame_;

// tf stuff
tf::TransformBroadcaster *tfB_;
tf::StampedTransform transform_;
tf::Quaternion tmp_;
tf::Quaternion imu_alignment_;
tf::Quaternion mag_north_correction_;
tf::Quaternion fixed_yaw_rotation_by_pi(0.0, 0.0, 1.0, 0);

std::vector<double> imu_alignment_rpy_(3, 0.0);
double mag_north_correction_yaw_ = 0;

// odom stuff
bool p_publish_odom_;
std::string p_odom_topic_name_;
ros::Publisher *odom_pub_;
nav_msgs::Odometry odom_msg_;

// gps stuff
geometry_msgs::Pose odom_from_gps;
geometry_msgs::Pose initial_gps;
geometry_msgs::Pose last_heading_gps_pose;
bool first_fix_received = false;
bool p_publish_gps_translation = true;
bool p_apply_gps_heading_correction = false;
double p_gps_heading_correction_weight = 0.1;
double p_gps_heading_min_dist = 3;

#ifndef TF_MATRIX3x3_H
typedef btScalar tfScalar;
namespace tf { typedef btMatrix3x3 Matrix3x3; }
#endif


void imuMsgCallback(const sensor_msgs::Imu &imu_msg) {
    tf::quaternionMsgToTF(imu_msg.orientation, tmp_);

    tmp_ = mag_north_correction_ * tmp_ * imu_alignment_;
    tmp_.normalize();

    transform_.setRotation(tmp_);
    transform_.setOrigin(tf::Vector3(odom_from_gps.position.x,odom_from_gps.position.y,odom_from_gps.position.z));
    transform_.stamp_ = imu_msg.header.stamp;

    tfB_->sendTransform(transform_);

    if (p_publish_odom_) {
        geometry_msgs::Quaternion quat_msg;
        tf::quaternionTFToMsg(tmp_, quat_msg);
        odom_msg_.pose.pose.orientation = quat_msg;
        odom_msg_.pose.pose.position = odom_from_gps.position;
        odom_msg_.header.stamp = imu_msg.header.stamp;
        odom_pub_->publish(odom_msg_);
    }
}

void fixMsgCallback(const nav_msgs::Odometry &gps_odom_msg) {
    // catch and save the initial message
    if(!first_fix_received){
        initial_gps = gps_odom_msg.pose.pose;
        first_fix_received = true;
    }

    // evaluate current pose in local frame
    geometry_msgs::Pose current_pose;
    current_pose.position.x = gps_odom_msg.pose.pose.position.x - initial_gps.position.x;
    current_pose.position.y = gps_odom_msg.pose.pose.position.y - initial_gps.position.y;
    current_pose.position.z = gps_odom_msg.pose.pose.position.z - initial_gps.position.z;

    // publish pose if asked for
    if(p_publish_gps_translation) {
        odom_from_gps = current_pose;
    }else{
        odom_from_gps.position.x = 0;
        odom_from_gps.position.y = 0;
        odom_from_gps.position.z = 0;
    }

    // use the pose to estimate heading, if asked for
    if(p_apply_gps_heading_correction){
        std::stringstream debug;
        double current_gps_heading = 0;
        double dist_since_last_pose = sqrt(
                pow(current_pose.position.x-last_heading_gps_pose.position.x,2) +
                pow(current_pose.position.y-last_heading_gps_pose.position.y,2)
                );
        if (dist_since_last_pose>=p_gps_heading_min_dist){
            current_gps_heading = atan2(current_pose.position.y - last_heading_gps_pose.position.y,
                                        current_pose.position.x - last_heading_gps_pose.position.x);


            tfScalar yaw_imu, pitch_imu, roll_imu;
            tf::Matrix3x3 mat(tmp_);
            mat.getEulerYPR(yaw_imu, pitch_imu, roll_imu);

            tf::Quaternion attitude_by_gps;
            attitude_by_gps.setRPY(roll_imu, pitch_imu, (tfScalar) current_gps_heading );

            // For skidoo: we are actually moving backwards, so we need to rotate the attitude by 180 degs around z
            attitude_by_gps = fixed_yaw_rotation_by_pi * attitude_by_gps;


            // Difference quaternion between the imu and gps
            tf::Quaternion heading_difference = attitude_by_gps * tmp_.inverse();

            // Apply with weight
            tfScalar yaw_correction_angle = heading_difference.getAngle();
            tf::Vector3 yaw_correction_axis = heading_difference.getAxis();

            if(yaw_correction_angle >= 3.14159265359){   //whoops, the quaternion 4*pi periodicidy problem...
                heading_difference = heading_difference * -1;   //this is the same rotation, but the shorter direction
                yaw_correction_angle = heading_difference.getAngle();
                yaw_correction_axis = heading_difference.getAxis();
            }

            // check the values
            //tfScalar yaw_gps, pitch_gps, roll_gps;
            //mat = tf::Matrix3x3(heading_difference);
            //mat.getEulerYPR(yaw_gps, pitch_gps, roll_gps);
            //
            //debug << "Yaw difference: " << yaw_gps*180.0/3.1416 << ", Corr angle: " << yaw_correction_angle*180.0/3.1416 << std::endl;
            //ROS_INFO(debug.str().c_str());
            // end debug

            yaw_correction_angle = yaw_correction_angle * p_gps_heading_correction_weight; // applying only a part of the rotation

            heading_difference.setRotation(yaw_correction_axis, yaw_correction_angle);

            mag_north_correction_ = heading_difference * mag_north_correction_;
            mag_north_correction_.normalize();


            // dont forget to remember the last pose
            last_heading_gps_pose = current_pose;
        }
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, ROS_PACKAGE_NAME);

    ros::NodeHandle n;
    ros::NodeHandle pn("~");

    // Load params
    pn.param("odom_frame", p_odom_frame_, std::string("odom"));
    pn.param("base_frame", p_base_frame_, std::string("base_link"));
    pn.param("publish_odom", p_publish_odom_, false);
    pn.param("publish_gps_translation", p_publish_gps_translation, true);
    pn.param("apply_gps_heading_correction", p_apply_gps_heading_correction, false);
    pn.param("gps_heading_correction_weight", p_gps_heading_correction_weight, 0.1);
    pn.param("gps_heading_min_dist", p_gps_heading_min_dist, 3.0);
    pn.param("odom_topic_name", p_odom_topic_name_, std::string("imu_odom"));


    // Quaternion for IMU alignment
    if (!pn.getParam("imu_alignment_rpy", imu_alignment_rpy_)) {
        ROS_WARN("Parameter imu_alignment_rpy is not a list of three numbers, setting default 0,0,0");
    } else {
        if (imu_alignment_rpy_.size() != 3) {
            ROS_WARN("Parameter imu_alignment_rpy is not a list of three numbers, setting default 0,0,0");
            imu_alignment_rpy_.assign(3, 0.0);
        }
    }

    // Quaternion for Magnetic North correction
    if (!pn.getParam("mag_north_correction_yaw", mag_north_correction_yaw_)) {
        ROS_WARN("Parameter mag_north_correction_yaw is not a double, setting default 0 radians");
    }


    // Evaluate alignment quternion
    imu_alignment_.setRPY(imu_alignment_rpy_[0],
                          imu_alignment_rpy_[1],
                          imu_alignment_rpy_[2]);

    // Evaluate nag. north corr. quternion
    mag_north_correction_.setRPY(0.0,
                                 0.0,
                                 mag_north_correction_yaw_);

    // Prepare the transform, set the origin to zero
    tfB_ = new tf::TransformBroadcaster();
    transform_.getOrigin().setX(0.0);
    transform_.getOrigin().setY(0.0);
    transform_.getOrigin().setZ(0.0);
    transform_.frame_id_ = p_odom_frame_;
    transform_.child_frame_id_ = p_base_frame_;

    // If odom required, advertize the publisher and prepare the constant parts of the message
    if (p_publish_odom_) {
        odom_pub_ = new ros::Publisher(n.advertise<nav_msgs::Odometry>(p_odom_topic_name_, 10));
        odom_msg_.header.frame_id = p_odom_frame_;

        //set the position
        odom_msg_.pose.pose.position.x = 0;
        odom_msg_.pose.pose.position.y = 0;
        odom_msg_.pose.pose.position.z = 0;

        //set the velocity
        odom_msg_.child_frame_id = p_base_frame_;
        odom_msg_.twist.twist.linear.x = 0;
        odom_msg_.twist.twist.linear.y = 0;
        odom_msg_.twist.twist.linear.z = 0;
        odom_msg_.twist.twist.angular.x = 0;     // This could be actuall set, but left for TODO (must be transformed by the alignment to the right frame)
        odom_msg_.twist.twist.angular.y = 0;
        odom_msg_.twist.twist.angular.z = 0;
    }


    // Subscribe the IMU and start the loop
    ros::Subscriber gps_odom_subscriber = n.subscribe("gps_odom_topic", 10, fixMsgCallback);
    ros::Subscriber imu_subscriber = n.subscribe("imu_topic", 10, imuMsgCallback);

    ros::spin();

    delete tfB_;
    delete odom_pub_;

    return 0;
}
