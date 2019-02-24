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

// frame names
std::string p_odom_frame_;
std::string p_base_frame_;

// tf stuff
tf::TransformBroadcaster* tfB_;
tf::StampedTransform transform_;
tf::Quaternion tmp_;
tf::Quaternion imu_alignment_;
std::vector<double> imu_alignment_rpy_(3,0.0);

// odom stuff
bool p_publish_odom_;
std::string p_odom_topic_name_;
ros::Publisher* odom_pub_;
nav_msgs::Odometry odom_msg_;


#ifndef TF_MATRIX3x3_H
  typedef btScalar tfScalar;
  namespace tf { typedef btMatrix3x3 Matrix3x3; }
#endif


void imuMsgCallback(const sensor_msgs::Imu& imu_msg)
{
  tf::quaternionMsgToTF(imu_msg.orientation, tmp_);

  tmp_ = imu_alignment_*tmp_;
  
  transform_.setRotation(tmp_);
  transform_.stamp_ = imu_msg.header.stamp;

  tfB_->sendTransform(transform_);

  if(p_publish_odom_)
  {
    geometry_msgs::Quaternion quat_msg;
    tf::quaternionTFToMsg(tmp_ , quat_msg);
    odom_msg_.pose.pose.orientation = quat_msg;
    odom_msg_.header.stamp = imu_msg.header.stamp;
    odom_pub_ -> publish(odom_msg_);
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
  pn.param("odom_topic_name", p_odom_topic_name_, std::string("imu_odom"));

  if(!pn.getParam("imu_alignment_rpy", imu_alignment_rpy_))
  {
    ROS_WARN("Parameter imu_alignment_rpy is not a list of three numbers, setting default 0,0,0");
  } else 
  {
    if(imu_alignment_rpy_.size() != 3)
    {
      ROS_WARN("Parameter imu_alignment_rpy is not a list of three numbers, setting default 0,0,0");
      imu_alignment_rpy_.assign(3, 0.0);
    }
  }

  
  // Evaluate alignment quternion
  imu_alignment_.setRPY(imu_alignment_rpy_[0],
                        imu_alignment_rpy_[1],
                        imu_alignment_rpy_[2]);   

  // Prepare the transform, set the origin to zero
  tfB_ = new tf::TransformBroadcaster();
  transform_.getOrigin().setX(0.0);
  transform_.getOrigin().setY(0.0);
  transform_.getOrigin().setZ(0.0);
  transform_.frame_id_ = p_odom_frame_;
  transform_.child_frame_id_ = p_base_frame_;

  // If odom required, advertize the publisher and prepare the constant parts of the message
  if(p_publish_odom_)
  {
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
  ros::Subscriber imu_subscriber = n.subscribe("imu_topic", 10, imuMsgCallback);

  ros::spin();

  delete tfB_;
  delete odom_pub_;

  return 0;
}
