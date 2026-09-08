#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <cmath>
#include <memory>
#include <optional>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Vector3.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

#define MISSED_MSG_SAFETY_MULTIPLIER 6.0

namespace norlab_imu_tools {
// Planar dead reckoning: yaw from integrating the gyro's z axis, translation from the wheel
// odometry's forward speed. No attitude source at all, so z, roll and pitch stay zero and the
// estimate keeps going when the altimeters drop out.
class Se2WheelGyroOdom : public rclcpp::Node {
  public:
    Se2WheelGyroOdom() : Node("se2_wheel_gyro_odom") {
        p_longest_expected_imu_period_ = declareExpectedPeriod("imu_expected_rate", 200.0);
        p_longest_expected_wheel_odom_period_ = declareExpectedPeriod("wheel_odom_expected_rate", 10.0);

        p_wheel_odom_vx_scale_ = this->declare_parameter<double>("wheel_odom_velocity_scale_x", 0.95);
        p_odom_frame_ = this->declare_parameter<std::string>("odom_frame", "odom");
        p_base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
        p_publish_tf_ = this->declare_parameter<bool>("publish_tf", false);

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "imu_topic", 10, std::bind(&Se2WheelGyroOdom::imuCallback, this, std::placeholders::_1));

        wheel_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "wheel_odom_topic", 10, std::bind(&Se2WheelGyroOdom::wheelOdomCallback, this, std::placeholders::_1));

        se2_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("se2_odom_topic", 10);

        if (p_publish_tf_) {
            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        }
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

        *last_imu_msg_stamp_ = current_stamp;

        if (!isFresh(last_wheel_odom_stamp_, current_stamp, p_longest_expected_wheel_odom_period_)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "No fresh wheel odometry message, assuming zero speed.");
            speed_x_base_link_ = 0.0;
        }

        x_ += speed_x_base_link_ * std::cos(yaw_) * dt;
        y_ += speed_x_base_link_ * std::sin(yaw_) * dt;

        tf2::Quaternion orientation_quat;
        orientation_quat.setRPY(0.0, 0.0, yaw_);

        publishOdometry(current_stamp, orientation_quat);

        if (p_publish_tf_) {
            broadcastTf(current_stamp, orientation_quat);
        }
    }

    void publishOdometry(const rclcpp::Time& timestamp, const tf2::Quaternion& odom_to_base_link_rotation) {
        nav_msgs::msg::Odometry odom_msg;

        odom_msg.header.stamp = timestamp;
        odom_msg.header.frame_id = p_odom_frame_;
        odom_msg.child_frame_id = p_base_frame_;

        odom_msg.pose.pose.position.x = x_;
        odom_msg.pose.pose.position.y = y_;
        odom_msg.pose.pose.position.z = 0.0;

        odom_msg.pose.pose.orientation = tf2::toMsg(odom_to_base_link_rotation);

        odom_msg.twist.twist.linear.x = speed_x_base_link_;
        odom_msg.twist.twist.linear.y = 0.0;
        odom_msg.twist.twist.linear.z = 0.0;

        se2_odom_pub_->publish(odom_msg);
    }

    void broadcastTf(const rclcpp::Time& timestamp, const tf2::Quaternion& odom_to_base_link_rotation) {
        geometry_msgs::msg::TransformStamped transform;

        transform.header.stamp = timestamp;
        transform.header.frame_id = p_odom_frame_;
        transform.child_frame_id = p_base_frame_;

        transform.transform.translation.x = x_;
        transform.transform.translation.y = y_;
        transform.transform.translation.z = 0.0;

        transform.transform.rotation = tf2::toMsg(odom_to_base_link_rotation);

        tf_broadcaster_->sendTransform(transform);
    }

    void wheelOdomCallback(const nav_msgs::msg::Odometry& wheel_odom_msg) {
        if (std::isnan(wheel_odom_msg.twist.twist.linear.x)) {
            RCLCPP_WARN(this->get_logger(), "Received Wheel Odometry message with NaN values, dropping");
            return;
        }

        speed_x_base_link_ = wheel_odom_msg.twist.twist.linear.x * p_wheel_odom_vx_scale_;
        last_wheel_odom_stamp_ = rclcpp::Time(wheel_odom_msg.header.stamp);
    }

  private:
    double declareExpectedPeriod(const std::string& parameter_name, double default_rate) {
        double rate = this->declare_parameter<double>(parameter_name, default_rate);

        if (rate <= 0.0) {
            throw std::invalid_argument("Zero or negative rate for " + parameter_name + " is a nonsense.");
        }

        return MISSED_MSG_SAFETY_MULTIPLIER / rate;
    }

    static bool isFresh(const std::optional<rclcpp::Time>& last_stamp, const rclcpp::Time& now, double longest_expected_period) {
        return last_stamp.has_value() && (now - *last_stamp).seconds() < longest_expected_period;
    }

    std::optional<rclcpp::Time> last_imu_msg_stamp_;
    std::optional<rclcpp::Time> last_wheel_odom_stamp_;
    double speed_x_base_link_ = 0.0;
    double yaw_ = 0.0;
    double x_ = 0.0;
    double y_ = 0.0;

    double p_longest_expected_imu_period_;
    double p_longest_expected_wheel_odom_period_;
    double p_wheel_odom_vx_scale_;
    std::string p_odom_frame_;
    std::string p_base_frame_;
    bool p_publish_tf_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_odom_sub_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr se2_odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};
}; // namespace norlab_imu_tools

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<norlab_imu_tools::Se2WheelGyroOdom>());
    rclcpp::shutdown();
    return 0;
}
