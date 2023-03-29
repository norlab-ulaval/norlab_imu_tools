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
        biasSub = this->create_subscription<geometry_msgs::msg::Vector3Stamped>("bias_topic_in", 10,
                                                                                std::bind(&imuBiasCompensatorNode::biasMsgCallback, this,
                                                                                           std::placeholders::_1));

        imuSubscription = this->create_subscription<sensor_msgs::msg::Imu>("imu_topic_in", 10,
                                                                           std::bind(&imuBiasCompensatorNode::imuMsgCallback, this,
                                                                                      std::placeholders::_1));
        imuCompensatedPub = this->create_publisher<sensor_msgs::msg::Imu>("imu_topic_out", 10);
    }
private:
    double xBias = 0.0;
    double yBias = 0.0;
    double zBias = 0.0;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuCompensatedPub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSubscription;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr biasSub;

    void biasMsgCallback(const geometry_msgs::msg::Vector3Stamped& biasMsg)
    {
        this->xBias = biasMsg.vector.x;
        this->yBias = biasMsg.vector.y;
        this->zBias = biasMsg.vector.z;
        RCLCPP_INFO(this->get_logger(), "Bias acquired.");
    }

    void imuMsgCallback(const sensor_msgs::msg::Imu &imuMsg) {
        sensor_msgs::msg::Imu imuMsgUnbiased;
        imuMsgUnbiased = imuMsg;
        imuMsgUnbiased.angular_velocity.x -= this->xBias;
        imuMsgUnbiased.angular_velocity.y -= this->yBias;
        imuMsgUnbiased.angular_velocity.z -= this->zBias ;
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
