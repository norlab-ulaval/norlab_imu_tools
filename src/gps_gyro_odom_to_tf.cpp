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
#include "tf/transform_listener.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Quaternion.h"


// frame names
std::string p_odom_frame_;
std::string p_base_frame_;

// tf stuff
tf::TransformBroadcaster *tfB_;
tf::StampedTransform transform_;
tf::Quaternion tmp_;
tf::Quaternion mag_heading_delta_quat_;
tf::Quaternion imu_alignment_;
tf::Quaternion imu_alignment_correction_;
tf::Quaternion mag_north_correction_;
bool mag_heading_collected = false;
bool mag_heading_applied   = false;
tfScalar mag_yaw = std::nan("");


std::vector<double> imu_correction_rpy_(3, 0.0);
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
bool first_fix_received = false;


#ifndef TF_MATRIX3x3_H
typedef btScalar tfScalar;
namespace tf { typedef btMatrix3x3 Matrix3x3; }
#endif


// this callback purpose is to collect initial guess of the mag. north
// after that, the subscriber will unsubscribe
void imuMsgMagCallback(const sensor_msgs::Imu &imu_msg) {
    tf::Quaternion mag_heading_quat_;

    tf::quaternionMsgToTF(imu_msg.orientation, mag_heading_quat_);

    if(mag_heading_collected) return;

    tfScalar pitch, roll;

    mag_heading_collected = true;
    mag_heading_quat_ = mag_north_correction_ * mag_heading_quat_ * imu_alignment_correction_ * imu_alignment_;

    tf::Matrix3x3 mat(mag_heading_quat_);
    mat.getEulerYPR(mag_yaw, pitch, roll);

    mag_heading_collected = true;
}

void imuMsgCallback(const sensor_msgs::Imu &imu_msg) {
    tf::quaternionMsgToTF(imu_msg.orientation, tmp_);

    tmp_ = tmp_ * imu_alignment_;

    // we have to compute the mag-yaw alignment quaternion to point out gyro-odometry to the desired heading
    if(!mag_heading_applied){
        tfScalar yaw_gyro, pitch_gyro, roll_gyro;

        tf::Matrix3x3 mat(tmp_);
        mat.getEulerYPR(yaw_gyro, pitch_gyro, roll_gyro);

        tf::Quaternion mag_heading_quat;
        mag_heading_quat.setRPY(roll_gyro, pitch_gyro, mag_yaw);

        mag_heading_delta_quat_ = mag_heading_quat * tmp_.inverse();
        mag_heading_delta_quat_.normalize();

        mag_heading_applied = true;

    }

    tmp_ = mag_heading_delta_quat_ * tmp_;

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

    odom_from_gps.position.x = gps_odom_msg.pose.pose.position.x - initial_gps.position.x;
    odom_from_gps.position.y = gps_odom_msg.pose.pose.position.y - initial_gps.position.y;
    odom_from_gps.position.z = gps_odom_msg.pose.pose.position.z - initial_gps.position.z;

}

int main(int argc, char **argv) {
    ros::init(argc, argv, ROS_PACKAGE_NAME);

    ros::NodeHandle n;
    ros::NodeHandle pn("~");

    // Load params
    pn.param("odom_frame", p_odom_frame_, std::string("odom"));
    pn.param("base_frame", p_base_frame_, std::string("base_link"));
    pn.param("publish_odom", p_publish_odom_, false);
    pn.param("odom_topic_name", p_odom_topic_name_, std::string("imu_odom"));

    // Quaternion for Magnetic North correction
    if (!pn.getParam("mag_north_correction_yaw", mag_north_correction_yaw_)) {
        ROS_WARN("Parameter mag_north_correction_yaw is not a double, setting default 0 radians");
    }

    bool imu_alignment_set_;
    // Quaternion for IMU alignment
    if (pn.hasParam("imu_alignment_rpy")) {
        imu_alignment_set_ = true;
        ROS_WARN("Parameter imu_alignment_rpy is deprecated. Set correct IMU->base_link orientation in your URDF file and use imu_correction_rpy instead.");
        if (!pn.getParam("imu_alignment_rpy", imu_alignment_rpy_)) {
            ROS_WARN("Parameter imu_alignment_rpy is not a list of three numbers, setting default 0,0,0");
        } else {
            if (imu_alignment_rpy_.size() != 3) {
                ROS_WARN("Parameter imu_alignment_rpy is not a list of three numbers, setting default 0,0,0");
                imu_alignment_rpy_.assign(3, 0.0);
            }
        }
    } else {
        imu_alignment_set_ = false;
    }

    // Quaternion for IMU correction
    if (!pn.getParam("imu_correction_rpy", imu_correction_rpy_))
    {
        ROS_WARN("Parameter imu_correction_rpy is not a list of three numbers, setting default 0,0,0");
    } else	{
        if (imu_correction_rpy_.size() != 3) {
            ROS_WARN("Parameter imu_correction_rpy is not a list of three numbers, setting default 0,0,0");
            imu_correction_rpy_.assign(3, 0.0);
        }
    }

	// get IMU frame_id
	auto imu_msg_ptr = ros::topic::waitForMessage<sensor_msgs::Imu>("imu_topic", n, ros::Duration(1.0));
	if (imu_msg_ptr == nullptr) {
		delete tfB_;
		delete odom_pub_;
		throw ros::Exception("Unable to get IMU frame_id");
	}

	std::string imu_frame = imu_msg_ptr->header.frame_id;
    // Get alignment quaternion
    if (!imu_alignment_set_) {
        // Get IMU->base_link tf
        tf::TransformListener tf_listener;
        tf::StampedTransform tf_imu_bl;

        try {
            tf_listener.waitForTransform(imu_frame, p_base_frame_, ros::Time(0), ros::Duration(1.0));
            tf_listener.lookupTransform(imu_frame, p_base_frame_, ros::Time(0), tf_imu_bl);
        } catch (tf::TransformException &ex) {
            ROS_ERROR("Unable to get tf between IMU and base_link (%s->%s)", imu_frame.c_str(), p_base_frame_.c_str());
            delete tfB_;
            delete odom_pub_;
            throw;
        }
        imu_alignment_ = tf_imu_bl.getRotation();
    } else {
        imu_alignment_.setRPY(imu_alignment_rpy_[0],
                              imu_alignment_rpy_[1],
                              imu_alignment_rpy_[2]);
    }

    // Evaluate correction quaternion
    imu_alignment_correction_.setRPY(imu_correction_rpy_[0],
                                     imu_correction_rpy_[1],
                                     imu_correction_rpy_[2]);


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
    ros::Subscriber imu_mag_subscriber = n.subscribe("imu_topic", 10, imuMsgMagCallback);
    ROS_INFO("Subcribed to gps and IMU with mag. heading");

    ros::Rate r(10);
    while(!mag_heading_collected && ros::ok()){
        ros::spinOnce();
        r.sleep();
    }

    ROS_INFO("Mag. heading collected, switching to attitude estimation excluding mag. data.");
    if(ros::ok()) {
        imu_mag_subscriber.shutdown();
        ros::Subscriber imu_subscriber = n.subscribe("imu_nomag_topic", 10, imuMsgCallback);
        ros::spin();
    }

    delete tfB_;
    delete odom_pub_;

    return 0;
}
