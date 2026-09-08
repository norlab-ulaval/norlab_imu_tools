#include "geometry_msgs/msg/quaternion_stamped.hpp"
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
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#define MISSED_MSG_SAFETY_MULTIPLIER 6.0

namespace norlab_imu_tools {
// Pure attitude prior. Position comes from extrapolating the ICP positions at constant
// velocity, orientation is the pressure-array attitude quaternion used verbatim. No gyro, no
// wheel odometry.
//
// attitude_from_pressure.py sets q_z to zero, so this prior hands the mapper a fixed heading
// for the whole run and ICP has to recover every bit of yaw on its own. That is the point of
// the experiment, not an oversight.
//
// Everything is assembled in the map frame, where the attitude actually lives, and composed
// into odom -> base_link only at publish time. Overwriting the rotation after composing
// through map -> odom would leave the mapper's accumulated yaw correction stacked on top of
// the attitude, which is no longer a pure attitude prior.
class AttitudeConstantVelocityOdom : public rclcpp::Node {
  public:
    AttitudeConstantVelocityOdom() : Node("attitude_constant_velocity_odom") {
        p_publish_rate_ = this->declare_parameter<double>("publish_rate", 100.0);
        if (p_publish_rate_ <= 0.0) {
            throw std::invalid_argument("Zero or negative publish_rate is a nonsense.");
        }

        // Deskewing makes the mapper look up TF at stamps spread across the current scan, which
        // are ahead of the newest ICP pose by a full scan period. tf2 refuses to extrapolate
        // past the newest transform it holds, so the TF has to be published ahead of the clock.
        p_lookahead_seconds_ = this->declare_parameter<double>("lookahead_seconds", 0.1);

        p_longest_expected_attitude_period_ = declareExpectedPeriod("attitude_expected_rate", 45.0);

        p_map_frame_ = this->declare_parameter<std::string>("map_frame", "map");
        p_odom_frame_ = this->declare_parameter<std::string>("odom_frame", "odom");
        p_base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
        p_publish_tf_ = this->declare_parameter<bool>("publish_tf", false);

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        icp_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "icp_odom_topic", 10,
            std::bind(&AttitudeConstantVelocityOdom::icpOdomCallback, this, std::placeholders::_1));

        attitude_sub_ = this->create_subscription<geometry_msgs::msg::QuaternionStamped>(
            "attitude_topic", 10,
            std::bind(&AttitudeConstantVelocityOdom::attitudeCallback, this, std::placeholders::_1));

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("attitude_constant_velocity_odom_topic", 10);

        if (p_publish_tf_) {
            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        }

        // create_wall_timer runs off the steady clock and would ignore use_sim_time. The bag
        // clock is the only clock that matters here.
        timer_ = rclcpp::create_timer(this, this->get_clock(), rclcpp::Duration::from_seconds(1.0 / p_publish_rate_),
                                      std::bind(&AttitudeConstantVelocityOdom::publishTimerCallback, this));
    }

    void icpOdomCallback(const nav_msgs::msg::Odometry& icp_odom_msg) {
        const rclcpp::Time current_stamp(icp_odom_msg.header.stamp);
        const auto& position = icp_odom_msg.pose.pose.position;

        if (std::isnan(position.x)) {
            RCLCPP_WARN(this->get_logger(), "Received ICP odometry message with NaN values, dropping");
            return;
        }

        const tf2::Vector3 current_position(position.x, position.y, position.z);

        if (last_position_.has_value()) {
            const double dt = (current_stamp - *last_position_stamp_).seconds();

            if (dt < 1e-7) {
                RCLCPP_WARN(this->get_logger(),
                            "Received ICP odometry message with negative or zero time increment, ignoring that one "
                            "and starting from the next one.");
                last_position_.reset();
                last_position_stamp_.reset();
                return;
            }

            velocity_ = (current_position - *last_position_) / dt;
        }

        last_position_ = current_position;
        last_position_stamp_ = current_stamp;
    }

    void attitudeCallback(const geometry_msgs::msg::QuaternionStamped& attitude_msg) {
        if (std::isnan(attitude_msg.quaternion.w)) {
            RCLCPP_WARN(this->get_logger(), "Received attitude message with NaN values, dropping");
            return;
        }

        tf2::fromMsg(attitude_msg.quaternion, attitude_);
        last_attitude_stamp_ = rclcpp::Time(attitude_msg.header.stamp);
    }

    void publishTimerCallback() {
        const rclcpp::Time target_stamp = this->now() + rclcpp::Duration::from_seconds(p_lookahead_seconds_);

        if (!isFresh(last_attitude_stamp_, target_stamp, p_longest_expected_attitude_period_)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "No fresh attitude message, holding the last known one.");
        }

        // Nothing to propagate yet: the mapper cannot register its first scan without an
        // odom -> base_link TF, and it cannot produce an ICP pose before that first scan. The
        // origin breaks the deadlock.
        tf2::Vector3 position(0.0, 0.0, 0.0);

        if (last_position_.has_value()) {
            const double elapsed = (target_stamp - *last_position_stamp_).seconds();
            position = *last_position_ + velocity_ * elapsed;
        }

        const tf2::Transform map_to_base(attitude_, position);

        // Latest available rather than the target stamp: the target is in the future and tf2
        // will not extrapolate. map -> odom moves slowly enough for that to be harmless.
        tf2::Transform odom_to_map = tf2::Transform::getIdentity();
        try {
            const geometry_msgs::msg::TransformStamped odom_to_map_msg =
                tf_buffer_->lookupTransform(p_odom_frame_, p_map_frame_, tf2::TimePointZero);
            tf2::fromMsg(odom_to_map_msg.transform, odom_to_map);
        } catch (const tf2::TransformException& exception) {
            // Expected until the mapper starts broadcasting map -> odom. Until then the two
            // frames coincide anyway.
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "No map -> odom transform yet, assuming the frames coincide: %s", exception.what());
        }

        const tf2::Transform odom_to_base = odom_to_map * map_to_base;

        publishOdometry(target_stamp, odom_to_base);

        if (p_publish_tf_) {
            broadcastTf(target_stamp, odom_to_base);
        }
    }

    void publishOdometry(const rclcpp::Time& timestamp, const tf2::Transform& odom_to_base) {
        nav_msgs::msg::Odometry odom_msg;

        odom_msg.header.stamp = timestamp;
        odom_msg.header.frame_id = p_odom_frame_;
        odom_msg.child_frame_id = p_base_frame_;

        tf2::toMsg(odom_to_base, odom_msg.pose.pose);

        odom_msg.twist.twist.linear.x = velocity_.x();
        odom_msg.twist.twist.linear.y = velocity_.y();
        odom_msg.twist.twist.linear.z = velocity_.z();

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

    static bool isFresh(const std::optional<rclcpp::Time>& last_stamp, const rclcpp::Time& now,
                        double longest_expected_period) {
        return last_stamp.has_value() && (now - *last_stamp).seconds() < longest_expected_period;
    }

    std::optional<tf2::Vector3> last_position_;
    std::optional<rclcpp::Time> last_position_stamp_;
    tf2::Vector3 velocity_ = tf2::Vector3(0.0, 0.0, 0.0);
    tf2::Quaternion attitude_ = tf2::Quaternion::getIdentity();
    std::optional<rclcpp::Time> last_attitude_stamp_;

    double p_publish_rate_;
    double p_lookahead_seconds_;
    double p_longest_expected_attitude_period_;
    std::string p_map_frame_;
    std::string p_odom_frame_;
    std::string p_base_frame_;
    bool p_publish_tf_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr icp_odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr attitude_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};
}; // namespace norlab_imu_tools

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<norlab_imu_tools::AttitudeConstantVelocityOdom>());
    rclcpp::shutdown();
    return 0;
}
