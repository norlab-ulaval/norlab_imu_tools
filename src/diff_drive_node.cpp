#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <chrono>

class DiffDriveOdometry : public rclcpp::Node {
public:
    DiffDriveOdometry() : Node("diff_drive_odometry") {
        // Parameters
        declare_parameter("wheel_base", 1.5);  // Distance between left and right wheels (meters)
        declare_parameter("wheel_radius", 0.3); // Wheel radius (meters) wheels = 0.3 Tracks 0.2
        declare_parameter("vel_covariance", 0.001); // Velocity covariance
        wheel_base_ = get_parameter("wheel_base").as_double();
        wheel_radius_ = get_parameter("wheel_radius").as_double();
        vel_covariance_ = get_parameter("vel_covariance").as_double();

        // Initialize pose and velocity
        x_ = y_ = theta_ = 0.0;
        left_velocity_ = right_velocity_ = 0.0;
        last_time_ = this->get_clock()->now();

        // Subscribers
        left_wheel_sub_ = create_subscription<std_msgs::msg::Float64>(
            "/warthog/platform/motor/left/status/velocity", 10,
            [this](const std_msgs::msg::Float64::SharedPtr msg) {
                left_velocity_ = msg->data * wheel_radius_; // Convert to m/s
            });

        right_wheel_sub_ = create_subscription<std_msgs::msg::Float64>(
            "/warthog/platform/motor/right/status/velocity", 10,
            [this](const std_msgs::msg::Float64::SharedPtr msg) {
                right_velocity_ = msg->data * wheel_radius_; // Convert to m/s
            });

        // Publisher
        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/warthog/platform/better_odom", 10);

        // Timer to update odometry
        timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&DiffDriveOdometry::update_odometry, this));
    }

private:
    // Parameters
    double wheel_base_;
    double wheel_radius_;
    double vel_covariance_;

    // State variables
    double x_, y_, theta_;
    double left_velocity_, right_velocity_;
    rclcpp::Time last_time_;

    // ROS interfaces
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr left_wheel_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr right_wheel_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void update_odometry() {
        rclcpp::Time current_time = this->get_clock()->now();
        double dt = (current_time - last_time_).seconds();
        std::cout << "dt: " << dt << std::endl;
        if (dt == 0.0) return;

        // Compute linear and angular velocities
        double v = (right_velocity_ + left_velocity_) / 2.0;
        double omega = (right_velocity_ - left_velocity_) / wheel_base_;

        // Update pose using differential drive kinematics
        x_ += v * dt * cos(theta_);
        y_ += v * dt * sin(theta_);
        theta_ += omega * dt;

        // Create quaternion from yaw
        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);

        // Publish odometry message
        auto odom_msg = nav_msgs::msg::Odometry();
        odom_msg.header.stamp = current_time;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";

        odom_msg.pose.pose.position.x = x_;
        odom_msg.pose.pose.position.y = y_;
        odom_msg.pose.pose.position.z = 0.0;
        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_msg.pose.pose.orientation.w = q.w();

        odom_msg.twist.twist.linear.x = v;
        odom_msg.twist.twist.angular.z = omega;

        // Set covariance
        odom_msg.twist.covariance[0] = vel_covariance_;
        odom_msg.twist.covariance[7] = vel_covariance_;
        odom_msg.twist.covariance[14] = vel_covariance_;
        odom_msg.twist.covariance[21] = vel_covariance_;
        odom_msg.twist.covariance[28] = vel_covariance_;
        odom_msg.twist.covariance[35] = vel_covariance_;

        odom_pub_->publish(odom_msg);

        // Update time
        last_time_ = current_time;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DiffDriveOdometry>());
    rclcpp::shutdown();
    return 0;
}
