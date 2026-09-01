#include "geometry_msgs/msg/quaternion_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <cmath>
#include <memory>
#include <optional>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Vector3.hpp>
#include <tf2/convert.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

#define MISSED_MSG_SAFETY_MULTIPLIER 6.0

namespace norlab_imu_tools {
class AltiGyroWheelOdom : public rclcpp::Node {
  public:
    AltiGyroWheelOdom() : Node("alti_gyro_wheel_odom") {
        p_longest_expected_imu_period_ = declareExpectedPeriod("imu_expected_rate", 200.0);
        p_longest_expected_attitude_period_ = declareExpectedPeriod("attitude_expected_rate", 45.0);
        p_longest_expected_wheel_odom_period_ = declareExpectedPeriod("wheel_odom_expected_rate", 10.0);

        attitude_sub_ = this->create_subscription<geometry_msgs::msg::QuaternionStamped>(
            "attitude_topic", 10, std::bind(&AltiGyroWheelOdom::attitudeCallback, this, std::placeholders::_1));

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "imu_topic", 10, std::bind(&AltiGyroWheelOdom::imuCallback, this, std::placeholders::_1));

        wheel_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "wheel_odom_topic", 10, std::bind(&AltiGyroWheelOdom::wheelOdomCallback, this, std::placeholders::_1));

        alti_gyro_wheel_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("alti_gyro_wheel_odom_topic", 10);

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    void imuCallback(const sensor_msgs::msg::Imu& imu_msg) {
        const rclcpp::Time current_stamp(imu_msg.header.stamp);

        if (std::isnan(imu_msg.angular_velocity.z)) {
            RCLCPP_WARN(this->get_logger(), "Received IMU message with NaN values, dropping");
            return;
        }

        if (!last_imu_msg_stamp_.has_value()) {
            last_imu_msg_stamp_ = current_stamp;
            return;
        }

        const double dt = (current_stamp - *last_imu_msg_stamp_).seconds();

        if (dt < 1e-7) {
            RCLCPP_WARN(this->get_logger(),
                        "Received IMU message with negative or zero time increment, ignoring that one and starting "
                        "from the next one.");
            last_imu_msg_stamp_.reset();
            return;
        }

        if (dt >= p_longest_expected_imu_period_) {
            RCLCPP_WARN(this->get_logger(),
                        "Received IMU message with too much delay after the previous one. Ignoring that and starting "
                        "from new.");
            last_imu_msg_stamp_ = current_stamp;
            return;
        }

        yaw_ += imu_msg.angular_velocity.z * dt;
        heading_quat_.setRPY(0.0, 0.0, yaw_);

        *last_imu_msg_stamp_ = current_stamp;

        if (!isFresh(last_attitude_stamp_, current_stamp, p_longest_expected_attitude_period_)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "No fresh attitude message, not publishing odometry.");
            return;
        }

        if (!isFresh(last_wheel_odom_stamp_, current_stamp, p_longest_expected_wheel_odom_period_)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "No fresh wheel odometry message, assuming zero speed.");
            speed_x_base_link_ = 0.0;
        }

        tf2::Quaternion orientation_quat = (heading_quat_ * attitude_quat_).normalize();

        const tf2::Vector3 velocity_base_link = tf2::Vector3(speed_x_base_link_, 0.0, 0.0);
        const tf2::Vector3 velocity_world = tf2::quatRotate(orientation_quat, velocity_base_link);

        current_position_ += dt * velocity_world;

        publishOdometry(current_stamp, orientation_quat);
        broadcastTf(current_stamp, orientation_quat);
    }

    void publishOdometry(const rclcpp::Time& timestamp, const tf2::Quaternion& odom_to_base_link_rotation) {
        nav_msgs::msg::Odometry odom_msg;

        odom_msg.header.stamp = timestamp;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";

        odom_msg.pose.pose.position.x = current_position_.x();
        odom_msg.pose.pose.position.y = current_position_.y();
        odom_msg.pose.pose.position.z = current_position_.z();

        odom_msg.pose.pose.orientation = tf2::toMsg(odom_to_base_link_rotation);

        odom_msg.twist.twist.linear.x = speed_x_base_link_;
        odom_msg.twist.twist.linear.y = 0.0;
        odom_msg.twist.twist.linear.z = 0.0;

        alti_gyro_wheel_odom_pub_->publish(odom_msg);
    }

    void broadcastTf(const rclcpp::Time& timestamp, const tf2::Quaternion& odom_to_base_link_rotation) {
        geometry_msgs::msg::TransformStamped transform;

        transform.header.stamp = timestamp;
        transform.header.frame_id = "odom";
        transform.child_frame_id = "base_link";

        transform.transform.translation.x = current_position_.x();
        transform.transform.translation.y = current_position_.y();
        transform.transform.translation.z = current_position_.z();

        transform.transform.rotation = tf2::toMsg(odom_to_base_link_rotation);

        tf_broadcaster_->sendTransform(transform);
    }

    void attitudeCallback(const geometry_msgs::msg::QuaternionStamped& attitude_msg) {
        if (std::isnan(attitude_msg.quaternion.x) || std::isnan(attitude_msg.quaternion.y) || std::isnan(attitude_msg.quaternion.z) ||
            std::isnan(attitude_msg.quaternion.w)) {
            RCLCPP_WARN(this->get_logger(), "Received Attitude Quaternion message with NaN values, dropping");
            return;
        }

        tf2::fromMsg(attitude_msg.quaternion, attitude_quat_);
        last_attitude_stamp_ = rclcpp::Time(attitude_msg.header.stamp);
    }

    void wheelOdomCallback(const nav_msgs::msg::Odometry& wheel_odom_msg) {
        if (std::isnan(wheel_odom_msg.twist.twist.linear.x) || std::isnan(wheel_odom_msg.twist.twist.linear.y) ||
            std::isnan(wheel_odom_msg.twist.twist.linear.z)) {
            RCLCPP_WARN(this->get_logger(), "Received Wheel Odometry message with NaN values, dropping");
            return;
        }

        speed_x_base_link_ = wheel_odom_msg.twist.twist.linear.x;
        last_wheel_odom_stamp_ = rclcpp::Time(wheel_odom_msg.header.stamp);
    }

  private:
    double declareExpectedPeriod(const std::string& parameter_name, double default_rate) {
        double rate = default_rate;
        this->declare_parameter<double>(parameter_name, default_rate);
        this->get_parameter(parameter_name, rate);

        if (rate <= 0.0) {
            throw std::invalid_argument("Zero or negative rate for " + parameter_name + " is a nonsense.");
        }

        return MISSED_MSG_SAFETY_MULTIPLIER / rate;
    }

    static bool isFresh(const std::optional<rclcpp::Time>& last_stamp, const rclcpp::Time& now, double longest_expected_period) {
        return last_stamp.has_value() && (now - *last_stamp).seconds() < longest_expected_period;
    }

    std::optional<rclcpp::Time> last_imu_msg_stamp_;
    std::optional<rclcpp::Time> last_attitude_stamp_;
    std::optional<rclcpp::Time> last_wheel_odom_stamp_;
    double speed_x_base_link_ = 0.0;
    double yaw_ = 0.0;
    tf2::Vector3 current_position_ = tf2::Vector3(0.0, 0.0, 0.0);
    tf2::Quaternion attitude_quat_ = tf2::Quaternion(0.0, 0.0, 0.0, 1.0);
    tf2::Quaternion heading_quat_ = tf2::Quaternion(0.0, 0.0, 0.0, 1.0);

    double p_longest_expected_imu_period_;
    double p_longest_expected_attitude_period_;
    double p_longest_expected_wheel_odom_period_;

    rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr attitude_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_odom_sub_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr alti_gyro_wheel_odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};
}; // namespace norlab_imu_tools

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<norlab_imu_tools::AltiGyroWheelOdom>());
    rclcpp::shutdown();
    return 0;
}
