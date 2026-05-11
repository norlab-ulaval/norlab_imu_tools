#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

namespace norlab_imu_tools
{

class CommandOdomNode : public rclcpp::Node
{
public:
  CommandOdomNode()
  : Node("command_odom_node"), x_(0.0), y_(0.0), th_(0.0)
  {
    this->declare_parameter("odom_frame", "odom");
    this->declare_parameter("base_frame", "base_link");
    this->declare_parameter("publish_tf", false);

    odom_frame_ = this->get_parameter("odom_frame").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();
    publish_tf_ = this->get_parameter("publish_tf").as_bool();

    last_time_ = this->now();

    // The launch file remaps these:
    // "cmd_vel_topic" -> incoming Twist
    // "odom_out_topic" -> outgoing Odometry
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom_out_topic", 10);
    
    vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel_topic", 10, std::bind(&CommandOdomNode::velocity_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Command Odom Node started (Open-Loop Integration)");
  }

private:
  void velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    rclcpp::Time current_time = this->now();
    double dt = (current_time - last_time_).seconds();
    last_time_ = current_time;

    double vx = msg->linear.x;
    double vth = msg->angular.z;

    // Numerical integration (Euler)
    double delta_x = vx * cos(th_) * dt;
    double delta_y = vx * sin(th_) * dt;
    double delta_th = vth * dt;

    x_ += delta_x;
    y_ += delta_y;
    th_ += delta_th;

    // Create Odometry message
    auto odom = nav_msgs::msg::Odometry();
    odom.header.stamp = current_time;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;

    // Set position
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;
    
    tf2::Quaternion q;
    q.setRPY(0, 0, th_);
    odom.pose.pose.orientation = tf2::toMsg(q);

    // Set velocity
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.angular.z = vth;

    odom_pub_->publish(odom);
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  
  std::string odom_frame_;
  std::string base_frame_;
  bool publish_tf_;

  double x_, y_, th_;
  rclcpp::Time last_time_;
};

} // namespace norlab_imu_tools

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<norlab_imu_tools::CommandOdomNode>());
  rclcpp::shutdown();
  return 0;
}
