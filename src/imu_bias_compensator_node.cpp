#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <cmath>
#include <sstream>

class imuBiasCompensatorNode : public rclcpp::Node
{
public:
    imuBiasCompensatorNode() :
            Node("imu_bias_compensator_node")
    {
        gyroBiasSub = this->create_subscription<geometry_msgs::msg::Vector3Stamped>("bias_topic_in", 10,
                                                                                std::bind(&imuBiasCompensatorNode::gyroBiasMsgCallback, this,
                                                                                           std::placeholders::_1));
        
        accelBiasSub = this->create_subscription<geometry_msgs::msg::Vector3Stamped>("accel_bias_topic_in", 10,
                                                                                std::bind(&imuBiasCompensatorNode::accelBiasMsgCallback, this,
                                                                                           std::placeholders::_1));

        imuSubscription = this->create_subscription<sensor_msgs::msg::Imu>("imu_topic_in", 10,
                                                                           std::bind(&imuBiasCompensatorNode::imuMsgCallback, this,
                                                                                      std::placeholders::_1));
        imuCompensatedPub = this->create_publisher<sensor_msgs::msg::Imu>("imu_topic_out", 10);
    }
private:
    double xGyroBias = 0.0;
    double yGyroBias = 0.0;
    double zGyroBias = 0.0;

    double xAccelBias = 0.0;
    double yAccelBias = 0.0;
    double zAccelBias = 0.0;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuCompensatedPub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSubscription;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr gyroBiasSub;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr accelBiasSub;

    void gyroBiasMsgCallback(const geometry_msgs::msg::Vector3Stamped& biasMsg)
    {
        this->xGyroBias = biasMsg.vector.x;
        this->yGyroBias = biasMsg.vector.y;
        this->zGyroBias = biasMsg.vector.z;
        RCLCPP_INFO(this->get_logger(), "Gyro bias acquired.");
    }

    void accelBiasMsgCallback(const geometry_msgs::msg::Vector3Stamped& biasMsg)
    {
        this->xAccelBias = biasMsg.vector.x;
        this->yAccelBias = biasMsg.vector.y;
        this->zAccelBias = biasMsg.vector.z;
        RCLCPP_INFO(this->get_logger(), "Accel bias acquired.");
    }

    void imuMsgCallback(const sensor_msgs::msg::Imu &imuMsg) {
        sensor_msgs::msg::Imu imuMsgUnbiased;
        imuMsgUnbiased = imuMsg;
        imuMsgUnbiased.angular_velocity.x -= this->xGyroBias;
        imuMsgUnbiased.angular_velocity.y -= this->yGyroBias;
        imuMsgUnbiased.angular_velocity.z -= this->zGyroBias;

        imuMsgUnbiased.linear_acceleration.x -= this->xAccelBias;
        imuMsgUnbiased.linear_acceleration.y -= this->yAccelBias;
        imuMsgUnbiased.linear_acceleration.z -= this->zAccelBias;
        imuCompensatedPub->publish(imuMsgUnbiased);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<imuBiasCompensatorNode>());
    rclcpp::shutdown();
    return 0;
}
