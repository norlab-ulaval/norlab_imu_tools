#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include "rtf_sensors_msgs/msg/custom_pressure_temperature.hpp"
#include <sensor_msgs/msg/temperature.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <cmath>
#include <sstream>

class altitudeComputation : public rclcpp::Node
{
public:
    altitudeComputation() :
            Node("altitude_computation_node")
    {
        refPressureIn = this->create_subscription<sensor_msgs::msg::FluidPressure>("ref_pressure_in", 10,
                                                                                std::bind(&altitudeComputation::refPressureMsgCallback, this,
                                                                                          std::placeholders::_1));
        refTempIn = this->create_subscription<sensor_msgs::msg::Temperature>("ref_temp_in", 10,
                                                                                std::bind(&altitudeComputation::refTempMsgCallback, this,
                                                                                          std::placeholders::_1));
        sensorPressureIn = this->create_subscription<rtf_sensors_msgs::msg::CustomPressureTemperature>("sensor_pressure_in", 10,
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
    double P0 = 101325.0;

    bool is_first_altitude = true;
    bool is_first_msg = true;
    bool first_ref_press_msg_received = false;
    bool first_ref_temp_msg_received = false;
    bool is_first_msg_ref_press = true;
    bool is_first_msg_ref_temp = true;

    double initial_altitude = 0.0;

    std::string formula;
    sensor_msgs::msg::Temperature lastRefTempMeasurement;
    std::mutex lastRefTempMutex;
    sensor_msgs::msg::FluidPressure lastRefPressureMeasurement;
    std::mutex lastRefPressureMutex;

    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr altitudePub;
    rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr refPressureIn;
    rclcpp::Subscription<sensor_msgs::msg::Temperature>::SharedPtr refTempIn;
    rclcpp::Subscription<rtf_sensors_msgs::msg::CustomPressureTemperature>::SharedPtr sensorPressureIn;

    void pressureMsgCallback(const rtf_sensors_msgs::msg::CustomPressureTemperature &pressure_msg)
    {
        if (this->is_first_msg)
        {
            this->P0 = pressure_msg.pressure;
        }
        double P = pressure_msg.pressure;
        double altitude = 0;
        this->lastRefPressureMutex.lock();
        double localRefPressure = this->lastRefPressureMeasurement.fluid_pressure;
        this->lastRefPressureMutex.unlock();
        this->lastRefTempMutex.lock();
        double localRefTemperature = this->lastRefTempMeasurement.temperature;
        this->lastRefTempMutex.unlock();
        if (this->first_ref_press_msg_received and this->first_ref_temp_msg_received)
        {
            if (this->formula == "barometric")
            {
                double exponent_part = std::pow(P/localRefPressure, (this->R*this->Lb)/(this->g*this->M));
                altitude = this->hb - (((localRefTemperature + 273.15)/this->Lb)*(exponent_part - 1)); //Temperature needs to be in Kelvin
            }
            else if (this->formula == "hypsometric")
            {
                // assuming the virtual temperature is the temperature measured by the dps sensor. in Kelvins
                double exponent_part = std::pow(P/this->Pb, (this->R*this->Lb)/(this->g*this->M));
                altitude = this->hb - ((this->Tb/this->Lb)*(exponent_part - 1));
            }
            else
            {
                altitude = ((this->R * (localRefTemperature + 273.15))/this->g) * std::log(this->P0/P);
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
    }
    void refTempMsgCallback(const sensor_msgs::msg::Temperature &temp_msg)
    {
        this->lastRefTempMutex.lock();
        this->lastRefTempMeasurement = temp_msg;
        this->lastRefTempMutex.unlock();
        if (this->is_first_msg_ref_temp)
        {
            this->first_ref_temp_msg_received = true;
            this->is_first_msg_ref_temp = false;
        }
    }
    void refPressureMsgCallback(const sensor_msgs::msg::FluidPressure &pressure_msg)
    {
        this->lastRefPressureMutex.lock();
        this->lastRefPressureMeasurement = pressure_msg;
        this->lastRefPressureMutex.unlock();
        if (this->is_first_msg_ref_press)
        {
            this->first_ref_press_msg_received = true;
            this->is_first_msg_ref_press = false;
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<altitudeComputation>());
    rclcpp::shutdown();
    return 0;
}
