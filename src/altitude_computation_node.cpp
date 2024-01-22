#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/temperature.hpp>
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
        tempIn = this->create_subscription<sensor_msgs::msg::Temperature>("temp_in", 10,
                                                                                std::bind(&altitudeComputation::tempMsgCallback, this,
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
    double P0 = 101325.0;

    bool is_first_altitude = true;
    bool is_first_msg = true;

    double initial_altitude = 0.0;

    std::string formula;
    sensor_msgs::msg::Temperature lastTempMeasurement;
    std::mutex lastTempMutex;

    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr altitudePub;
    rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr pressureIn;
    rclcpp::Subscription<sensor_msgs::msg::Temperature>::SharedPtr tempIn;

    void pressureMsgCallback(const sensor_msgs::msg::FluidPressure &pressure_msg)
    {
        if (this->is_first_msg)
        {
            this->P0 = pressure_msg.fluid_pressure;
        }
        double P = pressure_msg.fluid_pressure;
        double altitude = 0;
        if (this->formula == "barometric")
        {
            double exponent_part = std::pow(P/this->Pb, (this->R*this->Lb)/(this->g*this->M));
            altitude = this->hb - ((this->Tb/this->Lb)*(exponent_part - 1));
        }
        else if (this->formula == "hypsometric")
        {
            // assuming the virtual temperature is the temperature measured by the dps sensor.
            double exponent_part = std::pow(P/this->Pb, (this->R*this->Lb)/(this->g*this->M));
            altitude = this->hb - ((this->Tb/this->Lb)*(exponent_part - 1));
        }
        else
        {
            this->lastTempMutex.lock();
            altitude = ((this->R * this->lastTempMeasurement.temperature)/this->g) * std::log(this->P0/P);
            this->lastTempMutex.unlock();
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
        if (this->formula == "hypsometric")
        {
            output_msg.point.z = altitude;
        }
        else
        {
            output_msg.point.z = altitude - this->initial_altitude;
        }
    	altitudePub->publish(output_msg);
    }
    void tempMsgCallback(const sensor_msgs::msg::Temperature &temp_msg)
    {
        this->lastTempMutex.lock();
        this->lastTempMeasurement = msg;
        this->lastTempMutex.unlock();
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<altitudeComputation>());
    rclcpp::shutdown();
    return 0;
}
