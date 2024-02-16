// Imu and Wheel odometry for the Clearpath Warthog
// Licence: BSD
// Laval University, NORLAB, 2019

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/wait_for_message.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/utils.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <cmath>
#include <sstream>

using namespace std::chrono_literals;
#define MISSED_ODOM_MSG_SAFETY_MULTIPLIER 6.0

class imuAndWheelOdomNode : public rclcpp::Node
{
public:
    imuAndWheelOdomNode():
            Node("imu_and_wheel_odom_node")
    {
//            ros::init(argc, argv, ROS_PACKAGE_NAME);
        double p_wheel_odom_expected_rate = 20.0;
        this->declare_parameter<std::string>("odom_frame", "odom");
        this->get_parameter("odom_frame", p_odom_frame_);

        this->declare_parameter<std::string>("altimeter_frame", "altimeter_link");
        this->get_parameter("altimeter_frame", p_altimeter_frame_);

        this->declare_parameter<std::string>("base_frame", "base_link");
        this->get_parameter("base_frame", p_base_frame_);

        this->declare_parameter<bool>("publish_odom", false);
        this->get_parameter("publish_odom", p_publish_odom_);

        this->declare_parameter<std::string>("odom_topic_name", "imu_odom");
        this->get_parameter("odom_topic_name", p_odom_topic_name_);

        // If odom required, advertize the publisher and prepare the constant parts of the message
        if(p_publish_odom_)
        {
            imuAndWheelOdomPublisher = this->create_publisher<nav_msgs::msg::Odometry>(p_odom_topic_name_, 2);
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

        this->declare_parameter<bool>("publish_translation", true);
        this->get_parameter("publish_translation", p_allow_translation);

        this->declare_parameter<double>("wheel_odom_velocity_scale_x", 1.0);
        this->get_parameter("wheel_odom_velocity_scale_x", p_wheel_odom_vx_scale);

        this->declare_parameter<double>("wheel_odom_expected_rate", 20.0);
        this->get_parameter("wheel_odom_expected_rate", p_wheel_odom_expected_rate);

        if(p_wheel_odom_expected_rate <= 0)
        {
            throw "Zero or negative rate for wheel odometry is a nonsense.";
        }
        p_longest_expected_input_odom_period = (1.0 * MISSED_ODOM_MSG_SAFETY_MULTIPLIER) / p_wheel_odom_expected_rate;

        // get IMU frame_id
        RCLCPP_DEBUG(this->get_logger(), "Waiting for IMU message...");
        sensor_msgs::msg::Imu imu_msg;
        std::string imu_frame;

        auto sub = this->create_subscription<sensor_msgs::msg::Imu>("imu_topic", 1, [](const std::shared_ptr<const sensor_msgs::msg::Imu>&) {});
        auto response =  rclcpp::wait_for_message<sensor_msgs::msg::Imu, int64_t, std::milli>(imu_msg, sub, this->get_node_options().context(), 5s);

        if(response)
        {
            imu_frame = imu_msg.header.frame_id;
        }
        else
        {
            throw rclcpp::exceptions::InvalidTopicNameError(this->get_namespace(),
                                                            "No IMU message received."
                                                            "\nCannot find the tf between base_link and IMU without the IMU frame name."
                                                            "\nPlease make sure the IMU messages are published.",
                                                            0);
        }

        // Quaternion for IMU alignment
        if(imu_frame.empty())
        {
            throw rclcpp::exceptions::NameValidationError("frame", this->get_namespace(), "IMU frame cannot be empty.", 0);
        }
        else
        { // Get IMU->base_link tf
            if(imu_frame[0] == '/')
            {
                throw rclcpp::exceptions::NameValidationError("frame", this->get_namespace(), "In ROS 2, IMU frame name cannot start with '/'", 0);
            }
            RCLCPP_DEBUG(this->get_logger(), "Waiting for transform between %s and base_link frames", imu_frame.c_str());

            std::unique_ptr<tf2_ros::Buffer> tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            std::shared_ptr<tf2_ros::TransformListener> tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
            geometry_msgs::msg::TransformStamped tf_imu_bl;
            try
            {
//					wait for the buffer to be filled
//					skipping this would make several
//					'Warning: Invalid frame ID "imu" passed to canTransform argument target_frame - frame does not exist
//					 at line 93 in ./src/buffer_core.cpp' show up in the log.
                unsigned int ctr = 0;
                const unsigned int ctr_max = 10;
                auto sleep_duration_ms = 10ms;
                while(!tf_buffer->_frameExists(imu_frame))
                {
                    ctr++;
                    rclcpp::sleep_for(sleep_duration_ms);
                    if(ctr >= ctr_max)
                    {
                        throw tf2::TimeoutException("tf2 lookup timeout after: " +
                                                    std::to_string(ctr_max * sleep_duration_ms.count()) + " ms. IMU frame " + imu_frame + " doesn't exist");
                    }
                }
                RCLCPP_DEBUG(this->get_logger(), "Transform available after %d attempts", ctr);
                tf_imu_bl = tf_buffer->lookupTransform(imu_frame, p_base_frame_, this->now());
            }
            catch(const tf2::TransformException& ex)
            {
                RCLCPP_ERROR(this->get_logger(), "Unable to get tf between IMU and base_link (%s->%s): %s", imu_frame.c_str(),
                             p_base_frame_.c_str(), ex.what());
                throw ex;
            }
            tf2::Quaternion quat;
            tf2::fromMsg(tf_imu_bl.transform.rotation, quat);
            const tf2::Matrix3x3 matrix(quat);
            double roll, pitch, yaw;
            matrix.getRPY(roll, pitch, yaw);
            // Evaluate alignment quternion
            imu_alignment_.setRPY(roll, pitch, yaw);
            RCLCPP_INFO(this->get_logger(), "RPY from TF: %f, %f, %f", roll, pitch, yaw);
        }

        this->declare_parameter<double>("mag_north_correction_yaw", 0.0);
        is_imu_mag_north_correction_set = this->get_parameter("mag_north_correction_yaw", mag_north_correction_yaw_);

        // Quaternion for Magnetic North correction
        if(!is_imu_mag_north_correction_set)
        {
            RCLCPP_WARN(this->get_logger(), "Parameter mag_north_correction_yaw is not a double, setting default 0 radians");
        }


        // Evaluate nag. north corr. quternion
        mag_north_correction_.setRPY(0.0,
                                     0.0,
                                     mag_north_correction_yaw_);

        // Initialize the attitude
        current_attitude.setRPY(0.0, 0.0, 0.0);

        transform_.getOrigin().setX(0.0);
        transform_.getOrigin().setY(0.0);
        transform_.getOrigin().setZ(0.0);
        transform_.frame_id_ = p_odom_frame_;
        transform_msg_.child_frame_id = p_base_frame_;

        wheelOdomSubscription = this->create_subscription<nav_msgs::msg::Odometry>("wheel_odom_topic", 10,
                                                                                   std::bind(
                                                                                           &imuAndWheelOdomNode::wheelOdomMsgCallback,
                                                                                           this,
                                                                                           std::placeholders::_1));

        imuSubscription = this->create_subscription<sensor_msgs::msg::Imu>("imu_topic", 10,
                                                                           std::bind(
                                                                                   &imuAndWheelOdomNode::imuMsgCallback,
                                                                                   this,
                                                                                   std::placeholders::_1));

        altitudeSubscription = this->create_subscription<geometry_msgs::msg::PointStamped>("altitude_topic", 10,
                                                                           std::bind(
                                                                                   &imuAndWheelOdomNode::altitudeMsgCallback,
                                                                                   this,
                                                                                   std::placeholders::_1));

        this->declare_parameter<bool>("real_time", true);
        bool realTime;
        this->get_parameter("real_time", realTime);

        int messageQueueSize;
        if(realTime)
        {
            tfBuffer = std::unique_ptr<tf2_ros::Buffer>(new tf2_ros::Buffer(this->get_clock()));
            messageQueueSize = 1;
        }
        else
        {
            tfBuffer = std::unique_ptr<tf2_ros::Buffer>(new tf2_ros::Buffer(this->get_clock(), std::chrono::seconds(1000000)));
            messageQueueSize = 0;
        }
        tfListener = std::unique_ptr<tf2_ros::TransformListener>(new tf2_ros::TransformListener(*tfBuffer));
        tfBroadcaster = std::unique_ptr<tf2_ros::TransformBroadcaster>(new tf2_ros::TransformBroadcaster(*this));
    }

private:
    //frame names
    std::string p_odom_frame_;
    std::string p_altimeter_frame_;
    std::string p_base_frame_;
    //tf stuff
    tf2::Stamped<tf2::Transform> transform_;
    geometry_msgs::msg::TransformStamped transform_msg_;
    tf2::Quaternion tmp_;
    tf2::Quaternion current_attitude;
    tf2::Quaternion imu_alignment_;
    tf2::Quaternion mag_north_correction_;
    double mag_north_correction_yaw_;
    std::chrono::time_point<std::chrono::system_clock> stamp_;
    rclcpp::Time msg_stamp_;
    bool is_imu_alignment_set;
    bool is_imu_mag_north_correction_set;

    bool p_publish_odom_;
    std::string p_odom_topic_name_;
//        ros::Publisher *odom_pub_;
    nav_msgs::msg::Odometry odom_msg_;

    tf2::Vector3 current_position = tf2::Vector3(0.0, 0.0, 0.0);

    tf2::Vector3 current_linear_vel = tf2::Vector3(0.0, 0.0, 0.0);

    // input wheel odom stuff
    bool initial_wheel_odom_received = false;
    rclcpp::Time previous_w_odom_stamp;
    double p_wheel_odom_vx_scale = 1.0;
    double p_longest_expected_input_odom_period = MISSED_ODOM_MSG_SAFETY_MULTIPLIER / 20.0;

    // output odom stuff
    bool p_allow_translation = true;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;
    std::unique_ptr<tf2_ros::Buffer> tfBuffer;
    std::unique_ptr<tf2_ros::TransformListener> tfListener;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr imuAndWheelOdomPublisher;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheelOdomSubscription;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSubscription;
    // altitude stuff
    double firstAltitudeMeasurementCorrectFrame;
    double lastAltitudeMeasurementCorrectFrame;
    double lastAltitudeMeasurementAltiFrame;
    bool isFirstAltitude = true;
    std::mutex lastAltitudeAltiFrameMutex;
    std::mutex lastAltitudeMutex;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr altitudeSubscription;

//    #ifndef TF_MATRIX3x3_H
//        typedef btScalar tfScalar;
//    namespace tf { typedef btMatrix3x3 Matrix3x3; }
//    #endif

    void imuMsgCallback(const sensor_msgs::msg::Imu& imu_msg)
    {
//        tf2::quaternionMsgToTF(imu_msg.orientation, tmp_);
        tf2::fromMsg(imu_msg.orientation, tmp_);
        //tmp_ = tf2::Quaternion(0,0,0,1);
        if(std::isnan(tmp_.getX()) || std::isnan(tmp_.getY()) || std::isnan(tmp_.getZ()) || std::isnan(tmp_.getW()))
        {
            RCLCPP_WARN(this->get_logger(), "Received IMU message with NaN values, dropping");
            return;
        }

        tmp_ = mag_north_correction_ * tmp_ * imu_alignment_;
        tmp_.normalize();

        current_attitude = tmp_;
        transform_.setRotation(current_attitude);

        geometry_msgs::msg::TransformStamped currentAltiToImuTf = tfBuffer->lookupTransform(imu_msg.header.frame_id, p_altimeter_frame_, imu_msg.header.stamp,
                                                                                            std::chrono::milliseconds(100));
        geometry_msgs::msg::Pose AltiPoseInImuFrame;
        AltiPoseInImuFrame.position.x = currentAltiToImuTf.transform.translation.x;
        AltiPoseInImuFrame.position.y = currentAltiToImuTf.transform.translation.y;
        AltiPoseInImuFrame.position.z = currentAltiToImuTf.transform.translation.z;
        AltiPoseInImuFrame.orientation = currentAltiToImuTf.transform.rotation;

        geometry_msgs::msg::TransformStamped imuToOdomTfQuaternion;
        imuToOdomTfQuaternion.transform.rotation = imu_msg.orientation;

        geometry_msgs::msg::Pose altiPoseInOdomFrame;
        tf2::doTransform(AltiPoseInImuFrame, altiPoseInOdomFrame, imuToOdomTfQuaternion);

        geometry_msgs::msg::TransformStamped altiToOdomTf;
        lastAltitudeAltiFrameMutex.lock();
        altiToOdomTf.transform.translation.z = lastAltitudeMeasurementAltiFrame;
        lastAltitudeAltiFrameMutex.unlock();
        altiToOdomTf.transform.rotation = altiPoseInOdomFrame.orientation;

        geometry_msgs::msg::TransformStamped robotToAltiFrameTf = tfBuffer->lookupTransform(p_altimeter_frame_, p_base_frame_, imu_msg.header.stamp,
                                                                                            std::chrono::milliseconds(100));
        geometry_msgs::msg::Pose robotPoseInAltiFrame;
        robotPoseInAltiFrame.position.x = robotToAltiFrameTf.transform.translation.x;
        robotPoseInAltiFrame.position.y = robotToAltiFrameTf.transform.translation.y;
        robotPoseInAltiFrame.position.z = robotToAltiFrameTf.transform.translation.z;
        robotPoseInAltiFrame.orientation = robotToAltiFrameTf.transform.rotation;
        geometry_msgs::msg::Pose altitudeRobotInOdomFrame;
        tf2::doTransform(robotPoseInAltiFrame, altitudeRobotInOdomFrame, altiToOdomTf);
        if (isFirstAltitude)
        {
            firstAltitudeMeasurementCorrectFrame = altitudeRobotInOdomFrame.position.z;
            isFirstAltitude = false;
//            RCLCPP_INFO(this->get_logger(), "First Altitude measurement value: %f", firstAltitudeMeasurementCorrectFrame);
        }
        lastAltitudeMutex.lock();
        lastAltitudeMeasurementCorrectFrame = altitudeRobotInOdomFrame.position.z - firstAltitudeMeasurementCorrectFrame;
        current_position = tf2::Vector3(current_position.x(),
                                        current_position.y(),
                                        lastAltitudeMeasurementCorrectFrame);
//        RCLCPP_INFO(this->get_logger(), "Last Altitude measurement value: %f", lastAltitudeMeasurementCorrectFrame);
        lastAltitudeMutex.unlock();

        transform_.setOrigin(tf2::Vector3(current_position.x(), current_position.y(), current_position.z()));
        msg_stamp_ = rclcpp::Time(imu_msg.header.stamp);
        transform_.stamp_ = tf2::TimePoint(std::chrono::nanoseconds(msg_stamp_.nanoseconds()));
        tf2::convert(transform_, transform_msg_);
        transform_msg_.child_frame_id = p_base_frame_;


//        tfB_->sendTransform(transform_);
        tfBroadcaster->sendTransform(transform_msg_);

        if(p_publish_odom_)
        {
            geometry_msgs::msg::Quaternion quat_msg;
            tf2::convert(current_attitude, quat_msg);
            odom_msg_.pose.pose.orientation = quat_msg;

            odom_msg_.pose.pose.position.x = current_position.x();
            odom_msg_.pose.pose.position.y = current_position.y();
            odom_msg_.pose.pose.position.z = current_position.z();

            odom_msg_.twist.twist.linear.x = current_linear_vel.x();
            odom_msg_.twist.twist.linear.y = current_linear_vel.y();
            odom_msg_.twist.twist.linear.z = current_linear_vel.z();

            odom_msg_.header.stamp = imu_msg.header.stamp;
            imuAndWheelOdomPublisher->publish(odom_msg_);
        }
    }

    void wheelOdomMsgCallback(const nav_msgs::msg::Odometry& wheel_odom_msg)
    {

        //prepare variables
        tf2::Vector3 new_position;

        //check for nonsense data
        if(std::isnan(wheel_odom_msg.twist.twist.linear.x) ||
           std::isnan(wheel_odom_msg.twist.twist.linear.y) ||
           std::isnan(wheel_odom_msg.twist.twist.linear.z))
        {
            RCLCPP_WARN(this->get_logger(), "Received Wheel Odometry message with NaN values, dropping");
            return;
        }

        // to know our delta time step, we need the previous time stamp. The first odom message is used to initialize it
        if(!initial_wheel_odom_received)
        {
            previous_w_odom_stamp = wheel_odom_msg.header.stamp;
            initial_wheel_odom_received = true;
            return;
        }

        // compute and store new position if asked for
        if(p_allow_translation)
        {

            // body to world rotation
            tf2::Transform rotation_body_to_world;
            rotation_body_to_world.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));      // no translation
            rotation_body_to_world.setRotation(current_attitude);                          // current orientation


            // express the velocity in the world frame
            tf2::Vector3 velocity_in_world =
                    rotation_body_to_world * tf2::Vector3(wheel_odom_msg.twist.twist.linear.x * p_wheel_odom_vx_scale,
                                                          wheel_odom_msg.twist.twist.linear.y,
                                                          wheel_odom_msg.twist.twist.linear.z);
            // time increment
            double delta_t = (rclcpp::Time(wheel_odom_msg.header.stamp) - previous_w_odom_stamp).seconds();

            if(delta_t < 1e-7)
            {
                RCLCPP_WARN(this->get_logger(),
                            "Received wheel odometry message with negative or zero time increment, ignoring that one and starting from the next one.");
                initial_wheel_odom_received = false;
                return;
            }

            if(delta_t >= p_longest_expected_input_odom_period)
            {
                RCLCPP_WARN(this->get_logger(),
                            "Received wheel odometry message with too much delay after the previous one. Ignoring that and starting from new.");
                initial_wheel_odom_received = false;
                return;
            }



            // ... so the position increment is:
            new_position = current_position + delta_t * velocity_in_world;


            // update the current position and linear velocity
            lastAltitudeMutex.lock();
            current_position = tf2::Vector3(new_position.x(), new_position.y(), lastAltitudeMeasurementCorrectFrame);
            lastAltitudeMutex.unlock();

            current_linear_vel = tf2::Vector3(wheel_odom_msg.twist.twist.linear.x * p_wheel_odom_vx_scale,
                                              wheel_odom_msg.twist.twist.linear.y,
                                              wheel_odom_msg.twist.twist.linear.z);
        }

        previous_w_odom_stamp = wheel_odom_msg.header.stamp;
    }
    void altitudeMsgCallback(const geometry_msgs::msg::PointStamped& altitude_msg)
    {

        //prepare variables
        tf2::Vector3 new_position;

        //check for nonsense data
        if(std::isnan(altitude_msg.point.x) ||
           std::isnan(altitude_msg.point.y) ||
           std::isnan(altitude_msg.point.z))
        {
            RCLCPP_WARN(this->get_logger(), "Received Altitude message with NaN values, dropping");
            return;
        }
        lastAltitudeAltiFrameMutex.lock();
        lastAltitudeMeasurementAltiFrame = altitude_msg.point.z;
        lastAltitudeAltiFrameMutex.unlock();
    }
};

int main(int argc, char** argv)
{

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<imuAndWheelOdomNode>());
    rclcpp::shutdown();
    return 0;
}
