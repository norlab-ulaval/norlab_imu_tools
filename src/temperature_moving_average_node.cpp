#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <boost/circular_buffer.hpp>
#include <cmath>
#include <sstream>

int HORIZON = 25;

class temperatureMovingAverage : public rclcpp::Node
{
public:
    temperatureMovingAverage() :
            Node("temperature_moving_average_node")
    {
        tempIn = this->create_subscription<sensor_msgs::msg::Temperature>("temp_in", 10,
                                                                                std::bind(&temperatureMovingAverage::temperatureMsgCallback, this,
                                                                                          std::placeholders::_1));

        tempPub = this->create_publisher<sensor_msgs::msg::Temperature>("temp_out", 10);
        cb.set_capacity(HORIZON);
    }
private:
    boost::circular_buffer<double> cb;
    bool publishing = false;

    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr tempPub;
    rclcpp::Subscription<sensor_msgs::msg::Temperature>::SharedPtr tempIn;

    void temperatureMsgCallback(const sensor_msgs::msg::Temperature &temp_msg)
    {
        this->cb.push_back(temp_msg.temperature);
        if (this->cb.size() == this->cb.capacity())
        {
            if (!this->publishing)
            {
                RCLCPP_INFO(this->get_logger(), "Reached the targeted number of inputs for temperature moving average, starting to publish.");
                this->publishing = true;
            }
            // Publish the averaged message.
            double averaged_temperature = std::accumulate(this->cb.begin(), this->cb.end(), 0.0)/this->cb.capacity();
            sensor_msgs::msg::Temperature output_msg = temp_msg;
            output_msg.temperature = averaged_temperature;
            tempPub->publish(output_msg);
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<temperatureMovingAverage>());
    rclcpp::shutdown();
    return 0;
}
