// This node observes gyro bias on a given topic for a given time and then publishes the observation on a given topic.
// The system must be stationary during the estimation.
// Author: Vladimir Kubelka, kubelvla@gmail.com
// Maintainer: Haha, good luck
// Licence: BSD
// Laval University, NORLAB, 2019

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Vector3Stamped.h"
#include <cmath>
#include <sstream>



double angular_velocity_sum_x = 0.0;
double angular_velocity_sum_y = 0.0;
double angular_velocity_sum_z = 0.0;
int number_of_samples = 0;

// params stuff
ros::Publisher *bias_pub;
double observation_time = 15.0;
ros::Time observation_start;
bool observe_now = false;

void imuMsgCallback(const sensor_msgs::Imu &imu_msg) {
    if(observe_now)
    {
        if((ros::Time::now() - observation_start).toSec() <= observation_time)
        {
            angular_velocity_sum_x += imu_msg.angular_velocity.x;
            angular_velocity_sum_y += imu_msg.angular_velocity.y;
            angular_velocity_sum_z += imu_msg.angular_velocity.z;
            number_of_samples += 1;
        }
        else
            {
                geometry_msgs::Vector3Stamped bias_msg;
                bias_msg.header.stamp = ros::Time::now();
                bias_msg.vector.x = angular_velocity_sum_x / double(number_of_samples);
                bias_msg.vector.y = angular_velocity_sum_y / double(number_of_samples);
                bias_msg.vector.z = angular_velocity_sum_z / double(number_of_samples);
                (*bias_pub).publish(bias_msg);

                observe_now = false;
            }

    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, ROS_PACKAGE_NAME);

    ros::NodeHandle n;
    ros::NodeHandle pn("~");

    // Load params
    pn.param("observation_time", observation_time, 15.0);

    // Subscribe the IMU and start the loop
    ros::Subscriber imu_subscriber = n.subscribe("imu_topic_in", 10, imuMsgCallback);
    // Advertise the publisher
    bias_pub = new ros::Publisher(n.advertise<geometry_msgs::Vector3Stamped>("bias_topic_out", 10));

    //give the subscriber time to contact the publisher
    ros::Duration(1.0).sleep();
    observation_start = ros::Time::now();
    observe_now = true;

    ros::spin();

    return 0;
}
