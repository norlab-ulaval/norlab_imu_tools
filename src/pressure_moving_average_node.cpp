#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <boost/circular_buffer.hpp>
#include <cmath>
#include <sstream>

int HORIZON = 25;

class pressureMovingAverage : public rclcpp::Node
{
public:
    pressureMovingAverage() :
            Node("pressure_moving_average_node")
    {
        pressureIn = this->create_subscription<sensor_msgs::msg::FluidPressure>("pressure_in", 10,
                                                                                std::bind(&pressureMovingAverage::pressureMsgCallback, this,
                                                                                          std::placeholders::_1));

        pressurePub = this->create_publisher<sensor_msgs::msg::FluidPressure>("pressure_out", 10);
        cb.set_capacity(HORIZON);
    }
private:
    boost::circular_buffer<double> cb;
    bool publishing = false;

    rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr pressurePub;
    rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr pressureIn;

    void pressureMsgCallback(const sensor_msgs::msg::FluidPressure &pressure_msg)
    {
        this->cb.push_back(pressure_msg.fluid_pressure);
        if (this->cb.size() == this->cb.capacity())
        {
            if (!this->publishing)
            {
                RCLCPP_INFO(this->get_logger(), "Reached the targeted number of inputs for pressure moving average, starting to publish.");
                this->publishing = true;
            }
            // Publish the averaged message.
            double averaged_pressure = std::accumulate(this->cb.begin(), this->cb.end(), 0.0)/this->cb.capacity();
            sensor_msgs::msg::FluidPressure output_msg = pressure_msg;
            output_msg.fluid_pressure = averaged_pressure;
            pressurePub->publish(output_msg);
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<pressureMovingAverage>());
    rclcpp::shutdown();
    return 0;
}
