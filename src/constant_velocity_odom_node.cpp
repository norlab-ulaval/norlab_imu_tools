#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include <memory>
#include <optional>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2/LinearMath/Vector3.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

#define MISSED_MSG_SAFETY_MULTIPLIER 6.0

namespace norlab_imu_tools {
// Sensorless motion prior: take the last delta between two ICP poses and keep applying it.
// The mapper reads its prior from the odom -> base_link TF and publishes icp_odom in the map
// frame, so this node closes the loop on the mapper itself. Whatever the mapping achieves with
// this prior is the floor, what ICP can do with no odometry at all.
//
// The delta is taken between consecutive ICP poses in the map frame and integrated onto this
// node's own odom-frame pose, which is what KISS-ICP's prediction model does. Reconstructing
// the odom pose as map_to_odom^-1 * icp_odom instead looks tempting and is wrong: the mapper
// builds map -> odom out of the very TF this node broadcasts, so the two cancel and the node
// ends up measuring the velocity of its own extrapolation.
class ConstantVelocityOdom : public rclcpp::Node {
  public:
    ConstantVelocityOdom() : Node("constant_velocity_odom") {
        p_publish_rate_ = this->declare_parameter<double>("publish_rate", 100.0);
        if (p_publish_rate_ <= 0.0) {
            throw std::invalid_argument("Zero or negative publish_rate is a nonsense.");
        }

        // Deskewing makes the mapper look up TF at stamps spread across the current scan, which
        // are ahead of the newest ICP pose by a full scan period. tf2 refuses to extrapolate
        // past the newest transform it holds, so the TF has to be published ahead of the clock.
        p_lookahead_seconds_ = this->declare_parameter<double>("lookahead_seconds", 0.1);

        // Past this, the prior holds its last pose instead of scaling the delta by a factor
        // that keeps growing. A stalled mapper would otherwise send the prior to infinity.
        p_longest_expected_icp_period_ = declareExpectedPeriod("icp_expected_rate", 10.0);

        p_odom_frame_ = this->declare_parameter<std::string>("odom_frame", "odom");
        p_base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
        p_publish_tf_ = this->declare_parameter<bool>("publish_tf", false);

        icp_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "icp_odom_topic", 10, std::bind(&ConstantVelocityOdom::icpOdomCallback, this, std::placeholders::_1));

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("constant_velocity_odom_topic", 10);

        if (p_publish_tf_) {
            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        }

        // create_wall_timer runs off the steady clock and would ignore use_sim_time. The bag
        // clock is the only clock that matters here.
        const auto period = std::chrono::duration<double>(1.0 / p_publish_rate_);
        timer_ = rclcpp::create_timer(this, this->get_clock(), rclcpp::Duration::from_seconds(period.count()),
                                      std::bind(&ConstantVelocityOdom::publishTimerCallback, this));
    }

    void icpOdomCallback(const nav_msgs::msg::Odometry& icp_odom_msg) {
        const rclcpp::Time current_stamp(icp_odom_msg.header.stamp);

        tf2::Transform map_to_base;
        tf2::fromMsg(icp_odom_msg.pose.pose, map_to_base);

        if (std::isnan(map_to_base.getOrigin().x())) {
            RCLCPP_WARN(this->get_logger(), "Received ICP odometry message with NaN values, dropping");
            return;
        }

        if (last_map_to_base_.has_value()) {
            const double dt = (current_stamp - *last_pose_stamp_).seconds();

            if (dt < 1e-7) {
                RCLCPP_WARN(this->get_logger(),
                            "Received ICP odometry message with negative or zero time increment, ignoring that one "
                            "and starting from the next one.");
                last_map_to_base_.reset();
                last_pose_stamp_.reset();
                last_delta_.reset();
                last_delta_seconds_ = 0.0;
                return;
            }

            // Body-frame delta, so the extrapolation follows the same arc the robot was on
            // rather than a straight line in the odom frame.
            last_delta_ = last_map_to_base_->inverse() * map_to_base;
            last_delta_seconds_ = dt;

            last_pose_ = last_pose_.value_or(tf2::Transform::getIdentity()) * *last_delta_;
        }

        last_map_to_base_ = map_to_base;
        last_pose_stamp_ = current_stamp;
    }

    void publishTimerCallback() {
        const rclcpp::Time target_stamp = this->now() + rclcpp::Duration::from_seconds(p_lookahead_seconds_);

        // Nothing to propagate yet: the mapper cannot register its first scan without an
        // odom -> base_link TF, and it cannot produce an ICP pose before that first scan.
        // Identity breaks the deadlock.
        tf2::Transform pose = tf2::Transform::getIdentity();

        if (last_pose_.has_value()) {
            pose = *last_pose_;

            const double elapsed = (target_stamp - *last_pose_stamp_).seconds();

            if (last_delta_.has_value() && elapsed < p_longest_expected_icp_period_) {
                pose = pose * scaleTransform(*last_delta_, elapsed / last_delta_seconds_);
            } else if (last_delta_.has_value()) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                     "No fresh ICP pose, holding the last one instead of extrapolating.");
            }
        }

        publishOdometry(target_stamp, pose);

        if (p_publish_tf_) {
            broadcastTf(target_stamp, pose);
        }
    }

    void publishOdometry(const rclcpp::Time& timestamp, const tf2::Transform& odom_to_base) {
        nav_msgs::msg::Odometry odom_msg;

        odom_msg.header.stamp = timestamp;
        odom_msg.header.frame_id = p_odom_frame_;
        odom_msg.child_frame_id = p_base_frame_;

        tf2::toMsg(odom_to_base, odom_msg.pose.pose);

        if (last_delta_.has_value()) {
            // Twist belongs in child_frame_id, which is where the delta already lives.
            const tf2::Vector3 linear = last_delta_->getOrigin() / last_delta_seconds_;
            odom_msg.twist.twist.linear.x = linear.x();
            odom_msg.twist.twist.linear.y = linear.y();
            odom_msg.twist.twist.linear.z = linear.z();

            const tf2::Quaternion rotation = last_delta_->getRotation();
            const tf2::Vector3 angular = rotation.getAxis() * (rotation.getAngle() / last_delta_seconds_);
            odom_msg.twist.twist.angular.x = angular.x();
            odom_msg.twist.twist.angular.y = angular.y();
            odom_msg.twist.twist.angular.z = angular.z();
        }

        odom_pub_->publish(odom_msg);
    }

    void broadcastTf(const rclcpp::Time& timestamp, const tf2::Transform& odom_to_base) {
        geometry_msgs::msg::TransformStamped transform;

        transform.header.stamp = timestamp;
        transform.header.frame_id = p_odom_frame_;
        transform.child_frame_id = p_base_frame_;

        transform.transform = tf2::toMsg(odom_to_base);

        tf_broadcaster_->sendTransform(transform);
    }

  private:
    double declareExpectedPeriod(const std::string& parameter_name, double default_rate) {
        double rate = this->declare_parameter<double>(parameter_name, default_rate);

        if (rate <= 0.0) {
            throw std::invalid_argument("Zero or negative rate for " + parameter_name + " is a nonsense.");
        }

        return MISSED_MSG_SAFETY_MULTIPLIER / rate;
    }

    // The transform raised to the power k. Not a slerp: tf2's slerp is only defined between 0
    // and 1 and this extrapolates past the end of the interval. The translation is scaled
    // linearly rather than along the screw axis, so the path is a chord of the true arc. Over
    // one scan period the difference does not matter.
    static tf2::Transform scaleTransform(const tf2::Transform& transform, double k) {
        const tf2::Quaternion rotation = transform.getRotation();

        tf2::Quaternion scaled_rotation(rotation.getAxis(), rotation.getAngle() * k);
        scaled_rotation.normalize();

        return tf2::Transform(scaled_rotation, transform.getOrigin() * k);
    }

    std::optional<tf2::Transform> last_pose_;
    std::optional<tf2::Transform> last_map_to_base_;
    std::optional<rclcpp::Time> last_pose_stamp_;
    std::optional<tf2::Transform> last_delta_;
    double last_delta_seconds_ = 0.0;

    double p_publish_rate_;
    double p_lookahead_seconds_;
    double p_longest_expected_icp_period_;
    std::string p_odom_frame_;
    std::string p_base_frame_;
    bool p_publish_tf_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr icp_odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};
}; // namespace norlab_imu_tools

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<norlab_imu_tools::ConstantVelocityOdom>());
    rclcpp::shutdown();
    return 0;
}
