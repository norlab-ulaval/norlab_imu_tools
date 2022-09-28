// Imu and Wheel odometry for the Clearpath Warthog
// Licence: BSD
// Laval University, NORLAB, 2019

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
//#include <tf2/tf2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <cmath>
#include <sstream>

#define MISSED_ODOM_MSG_SAFETY_MULTIPLIER 6.0

class imuAndWheelOdomNode : public rclcpp::Node
{
public:
    imuAndWheelOdomNode() :
            Node("imu_and_wheel_odom_node")
        {
        //frame names
        std::string p_odom_frame_;
        std::string p_base_frame_;

        tfBroadcaster = std::unique_ptr<tf2_ros::TransformBroadcaster>(new tf2_ros::TransformBroadcaster(*this));

        imuAndWheelOdomPublisher = this->create_publisher<nav_msgs::msg::Odometry>("odom_out", 2);

        wheelOdomSubscription = this->create_subscription<nav_msgs::msg::Odometry>("wheel_odom_topic", 10,
                                                                                         std::bind(
                                                                                                 &imuAndWheelOdomNode::wheelOdomMsgCallback,
                                                                                                 this,
                                                                                                 std::placeholders::_1));

        imuSubscription = this->create_subscription<sensor_msgs::msg::PointCloud2>("imu_topic", 10,
                                                                                   std::bind(
                                                                                           &imuAndWheelOdomNode::imuMsgCallback,
                                                                                           this,
                                                                                           std::placeholders::_1));
        }

private:
    //tf stuff
    tf2::StampedTransform transform_;
    tf2::Quaternion tmp_;
    tf2::Quaternion current_attitude;
    tf2::Quaternion imu_alignment_;
    tf2::Quaternion mag_north_correction_;


    std::vector<double> imu_alignment_rpy_(

    3, 0.0);

    bool p_publish_odom_;
    std::string p_odom_topic_name_;
//        ros::Publisher *odom_pub_;
    nav_msgs::Odometry odom_msg_;

    tf2::Vector3 current_position(

    0.0,0.0,0.0);

    tf2::Vector3 current_linear_vel(

    0.0,0.0,0.0);


    double mag_north_correction_yaw_ = 0;

    // input wheel odom stuff
    bool initial_wheel_odom_received = false;
    rclcpp::Time previous_w_odom_stamp;
    double p_wheel_odom_vx_scale = 1.0;
    double p_longest_expected_input_odom_period = MISSED_ODOM_MSG_SAFETY_MULTIPLIER / 20.0;

    // output odom stuff
    bool p_allow_translation = true;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr imuAndWheelOdomPublisher;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheelOdomSubscription;

//#ifndef TF_MATRIX3x3_H
//    typedef btScalar tfScalar;
//namespace tf { typedef btMatrix3x3 Matrix3x3; }
//#endif

void imuMsgCallback(const sensor_msgs::Imu &imu_msg) {
    tf::quaternionMsgToTF(imu_msg.orientation, tmp_);

    if (std::isnan(tmp_.getX()) || std::isnan(tmp_.getY()) || std::isnan(tmp_.getZ()) || std::isnan(tmp_.getW())) {
        ROS_WARN("Received IMU message with NaN values, dropping");
        return;
    }

    tmp_ = mag_north_correction_ * tmp_ * imu_alignment_;
    tmp_.normalize();

    current_attitude = tmp_;
    transform_.setRotation(current_attitude);
    transform_.setOrigin(tf::Vector3(current_position.x(), current_position.y(), current_position.z()));
    transform_.stamp_ = imu_msg.header.stamp;

    tfB_->sendTransform(transform_);

    if (p_publish_odom_) {
        geometry_msgs::Quaternion quat_msg;
        tf::quaternionTFToMsg(current_attitude, quat_msg);
        odom_msg_.pose.pose.orientation = quat_msg;

        odom_msg_.pose.pose.position.x = current_position.x();
        odom_msg_.pose.pose.position.y = current_position.y();
        odom_msg_.pose.pose.position.z = current_position.z();

        odom_msg_.twist.twist.linear.x = current_linear_vel.x();
        odom_msg_.twist.twist.linear.y = current_linear_vel.y();
        odom_msg_.twist.twist.linear.z = current_linear_vel.z();

        odom_msg_.header.stamp = imu_msg.header.stamp;
        odom_pub_->publish(odom_msg_);
    }
    }

void wheelOdomMsgCallback(const nav_msgs::Odometry &wheel_odom_msg) {

    //prepare variables
    tf::Vector3 new_position;

    //check for nonsense data
    if (std::isnan(wheel_odom_msg.twist.twist.linear.x) ||
        std::isnan(wheel_odom_msg.twist.twist.linear.y) ||
        std::isnan(wheel_odom_msg.twist.twist.linear.z)) {
        ROS_WARN("Received Wheel Odometry message with NaN values, dropping");
        return;
    }

    // to know our delta time step, we need the previous time stamp. The first odom message is used to initialize it
    if (!initial_wheel_odom_received) {
        previous_w_odom_stamp = wheel_odom_msg.header.stamp;
        initial_wheel_odom_received = true;
        return;
    }

    // compute and store new position if asked for
    if (p_allow_translation) {

        // body to world rotation
        tf::Transform rotation_body_to_world;
        rotation_body_to_world.setOrigin(tf::Vector3(0.0, 0.0, 0.0));      // no translation
        rotation_body_to_world.setRotation(current_attitude);                          // current orientation


        // express the velocity in the world frame
        tf::Vector3 velocity_in_world =
                rotation_body_to_world * tf::Vector3(wheel_odom_msg.twist.twist.linear.x * p_wheel_odom_vx_scale,
                                                     wheel_odom_msg.twist.twist.linear.y,
                                                     wheel_odom_msg.twist.twist.linear.z);
        // for warthog, only x is expected non-zero

        // time increment
        double delta_t = (wheel_odom_msg.header.stamp - previous_w_odom_stamp).toSec();

        if (delta_t < 1e-7) {
            ROS_WARN(
                    "Received wheel odometry message with negative or zero time increment, ignoring that one and starting from the next one.");
            initial_wheel_odom_received = false;
            return;
        }

        if (delta_t >= p_longest_expected_input_odom_period) {
            ROS_WARN(
                    "Received wheel odometry message with too much delay after the previous one. Ignoring that and starting from new.");
            initial_wheel_odom_received = false;
            return;
        }



        // ... so the position increment is:
        new_position = current_position + delta_t * velocity_in_world;


        // update the current position and linear velocity
        current_position = new_position;
        current_linear_vel = tf::Vector3(wheel_odom_msg.twist.twist.linear.x * p_wheel_odom_vx_scale,
                                         wheel_odom_msg.twist.twist.linear.y,
                                         wheel_odom_msg.twist.twist.linear.z);

    }

    previous_w_odom_stamp = wheel_odom_msg.header.stamp;
}

}
};






int main(int argc, char **argv) {
    ros::init(argc, argv, ROS_PACKAGE_NAME);

    ros::NodeHandle n;
    ros::NodeHandle pn("~");
    double p_wheel_odom_expected_rate = 20.0;

    // Load params
    pn.param("odom_frame", p_odom_frame_, std::string("odom"));
    pn.param("base_frame", p_base_frame_, std::string("base_link"));
    pn.param("publish_odom", p_publish_odom_, false);
    pn.param("publish_translation", p_allow_translation, true);
    pn.param("wheel_odom_velocity_scale_x", p_wheel_odom_vx_scale, 1.0);
    pn.param("odom_topic_name", p_odom_topic_name_, std::string("imu_odom"));
    pn.param("wheel_odom_expected_rate", p_wheel_odom_expected_rate, 20.0);

    if(p_wheel_odom_expected_rate<=0)
    {
        throw "Zero or negative rate for wheel odometry is a nonsense.";
    }
    p_longest_expected_input_odom_period = (1.0*MISSED_ODOM_MSG_SAFETY_MULTIPLIER)/p_wheel_odom_expected_rate;

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

    // Initialize the attitude
    current_attitude.setRPY(0.0,0.0,0.0);

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
        odom_msg_.twist.twist.angular.x = 0;
        odom_msg_.twist.twist.angular.y = 0;
        odom_msg_.twist.twist.angular.z = 0;
    }


    // Subscribe the IMU and start the loop


    ros::spin();

    delete tfB_;
    delete odom_pub_;

    return 0;
}
