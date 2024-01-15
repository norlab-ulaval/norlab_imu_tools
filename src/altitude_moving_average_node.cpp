#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <boost/circular_buffer.hpp>
#include <cmath>
#include <sstream>

int HORIZON = 25;

class altitudeMovingAverage : public rclcpp::Node
{
public:
    altitudeMovingAverage() :
            Node("altitude_moving_average_node")
    {
        altitudeIn = this->create_subscription<geometry_msgs::msg::PointStamped>("altitude_in", 10,
                                                                                std::bind(&altitudeMovingAverage::altitudeMsgCallback, this,
                                                                                          std::placeholders::_1));

        altitudePub = this->create_publisher<geometry_msgs::msg::PointStamped>("altitude_out", 10);
        cb.set_capacity(HORIZON);
    }
private:
    boost::circular_buffer<double> cb;
    bool publishing = false;

    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr altitudePub;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr altitudeIn;

    void altitudeMsgCallback(const geometry_msgs::msg::PointStamped &altitude_msg)
    {
        this->cb.push_back(altitude_msg.point.z);
        if (this->cb.size() == this->cb.capacity())
        {
            if (!this->publishing)
            {
                RCLCPP_INFO(this->get_logger(), "Reached the targeted number of inputs for altitude moving average, starting to publish.");
                this->publishing = true;
            }
            // Publish the averaged message.
            double averaged_altitude = std::accumulate(this->cb.begin(), this->cb.end(), 0.0)/this->cb.capacity();
            geometry_msgs::msg::PointStamped output_msg = altitude_msg;
            output_msg.point.z = averaged_altitude;
            altitudePub->publish(output_msg);
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<altitudeMovingAverage>());
    rclcpp::shutdown();
    return 0;
}
