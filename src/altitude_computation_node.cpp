#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <cmath>
#include <sstream>

int HORIZON = 25;

class altitudeComputation : public rclcpp::Node
{
public:
    altitudeComputation() :
            Node("altitude_computation_node")
    {
        pressureIn = this->create_subscription<sensor_msgs::msg::FluidPressure>("pressure_in", 10,
                                                                                std::bind(&altitudeComputation::pressureMsgCallback, this,
                                                                                          std::placeholders::_1));
        altitudePub = this->create_publisher<geometry_msgs::msg::PointStamped>("altitude_out", 10);

        this->declare_parameter<std::string>("formula", "barometric");
        this->get_parameter("formula", formula);
    }
private:
    double Tb = 288.15;
    double Lb = 0.0065;
    double Pb = 101325.0;
    double hb = 0.0;
    double g = 9.80665;
    double R = 8.3144598;
    double M = 0.0289644;

    bool is_first_altitude = true;
    double initial_altitude = 0.0;

    std::string formula;

    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr altitudePub;
    rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr pressureIn;

    void pressureMsgCallback(const sensor_msgs::msg::FluidPressure &pressure_msg)
    {
        double P = pressure_msg.fluid_pressure;
        double altitude = 0;
        if (this->formula == "barometric")
        {
            double exponent_part = std::pow(P/this->Pb, (this->R*this->Lb)/(this->g*this->M));
            altitude = this->hb - ((this->Tb/this->Lb)*(exponent_part - 1));
        }
        else if (this->formula == "hypsometric")
        {
            double exponent_part = std::pow(P/this->Pb, (this->R*this->Lb)/(this->g*this->M));
            altitude = this->hb - ((this->Tb/this->Lb)*(exponent_part - 1));
        }
        else
        {
            double exponent_part = std::pow(P/this->Pb, (this->R*this->Lb)/(this->g*this->M));
            altitude = this->hb - ((this->Tb/this->Lb)*(exponent_part - 1));
        }
        if (this->is_first_altitude)
        {
            this->initial_altitude = altitude;
            this->is_first_altitude = false;
        }
        geometry_msgs::msg::PointStamped output_msg;
    	output_msg.header = pressure_msg.header;
    	output_msg.point.x = 0.0;
    	output_msg.point.y = 0.0;
    	output_msg.point.z = altitude - this->initial_altitude;
    	altitudePub->publish(output_msg);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<altitudeComputation>());
    rclcpp::shutdown();
    return 0;
}
