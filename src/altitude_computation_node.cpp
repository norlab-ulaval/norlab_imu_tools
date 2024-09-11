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
        refPressureInSetra = this->create_subscription<sensor_msgs::msg::FluidPressure>("ref_pressure_in_setra", 10,
                                                                                std::bind(&altitudeComputation::refPressureSetraMsgCallback, this,
                                                                                          std::placeholders::_1));
        refPressureInDPS = this->create_subscription<rtf_sensors_msgs::msg::CustomPressureTemperature>("ref_pressure_in_dps", 10,
                                                                                std::bind(&altitudeComputation::refPressureDPSMsgCallback, this,
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

        this->declare_parameter<bool>("use_setra", true);
        this->get_parameter("use_setra", useSetra);
    }
private:
    double Tb = 288.15;
    double Lb = 0.0065;
    double Pb = 101325.0;
    double hb = 0.0;
    double g = 9.80665;
    double R = 8.3144598;
    double Rd = 287.0;
    double M = 0.0289644;
    double P0 = 101325.0;

    bool is_first_altitude = true;
    bool is_first_msg = true;
    bool first_ref_pressure_msg_received_setra = false;
    bool first_ref_pressure_msg_received_dps = false;
    bool first_ref_temp_msg_received = false;
    bool is_first_msg_ref_pressure_setra = true;
    bool is_first_msg_ref_pressure_dps = true;
    bool is_first_msg_ref_temp = true;

    double initial_altitude = 0.0;

    std::string formula;
    bool useSetra;
    sensor_msgs::msg::Temperature lastRefTempMeasurement;
    std::mutex lastRefTempMutex;
    sensor_msgs::msg::FluidPressure lastRefPressureMeasurementSetra;
    std::mutex lastRefPressureSetraMutex;
    rtf_sensors_msgs::msg::CustomPressureTemperature lastRefPressureMeasurementDPS;
    std::mutex lastRefPressureDPSMutex;

    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr altitudePub;
    rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr refPressureInSetra;
    rclcpp::Subscription<sensor_msgs::msg::Temperature>::SharedPtr refTempIn;
    rclcpp::Subscription<rtf_sensors_msgs::msg::CustomPressureTemperature>::SharedPtr sensorPressureIn;
    rclcpp::Subscription<rtf_sensors_msgs::msg::CustomPressureTemperature>::SharedPtr refPressureInDPS;

    void pressureMsgCallback(const rtf_sensors_msgs::msg::CustomPressureTemperature &pressure_msg)
    {
        if (this->is_first_msg)
        {
            this->P0 = pressure_msg.pressure;
        }
        double P = pressure_msg.pressure;
        double altitude = 0;
        this->lastRefTempMutex.lock();
        double localRefTemperature = this->lastRefTempMeasurement.temperature;
        this->lastRefTempMutex.unlock();
        if (this->useSetra)
        {
            if (this->first_ref_pressure_msg_received_setra and this->first_ref_temp_msg_received)
            {
                this->lastRefPressureSetraMutex.lock();
                double localRefPressure = this->lastRefPressureMeasurementSetra.fluid_pressure;
                this->lastRefPressureSetraMutex.unlock();
                if (this->formula == "barometric")
                {
                    double exponent_part = std::pow(P/this->Pb, (this->R*this->Lb)/(this->g*this->M));
                    double exponentSetra = std::pow(localRefPressure/this->Pb, (this->R*this->Lb)/(this->g*this->M));
                    altitude =((this->Tb/this->Lb)*(exponentSetra - exponent_part)); //Temperature needs to be in Kelvin
                }
                else if (this->formula == "hypsometric")
                {
                    // assuming the virtual temperature is the temperature measured by the dps sensor. in Kelvins
                    altitude = ((this->Rd * localRefTemperature)/this->g)*std::log(localRefPressure/P);
                }
                else
                {
                    altitude = ((this->R * (localRefTemperature + 273.15))/this->g) * std::log(this->P0/P);
                }
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
        else
        {
            if (this->first_ref_pressure_msg_received_dps and this->first_ref_temp_msg_received)
            {
                this->lastRefPressureDPSMutex.lock();
                double localRefPressure = this->lastRefPressureMeasurementDPS.pressure;
                this->lastRefPressureDPSMutex.unlock();
                if (this->formula == "barometric")
                {
                    double exponent_dps = std::pow(localRefPressure/this->Pb, (this->R*this->Lb)/(this->g*this->M));
                    double exponent_part = std::pow(P/this->Pb, (this->R*this->Lb)/(this->g*this->M));
                    altitude = (this->Tb/this->Lb)*(exponent_dps - exponent_part); //Temperature needs to be in Kelvin
                }
                else if (this->formula == "hypsometric")
                {
                    // assuming the virtual temperature is the temperature measured by the dps sensor. in Kelvins
                    altitude = ((this->Rd * localRefTemperature)/this->g)*std::log(localRefPressure/P);
                }
                else
                {
                    altitude = ((this->R * (localRefTemperature + 273.15))/this->g) * std::log(this->P0/P);
                }
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
    void refPressureSetraMsgCallback(const sensor_msgs::msg::FluidPressure &pressure_msg)
    {
        this->lastRefPressureSetraMutex.lock();
        this->lastRefPressureMeasurementSetra = pressure_msg;
        this->lastRefPressureSetraMutex.unlock();
        if (this->is_first_msg_ref_pressure_setra)
        {
            this->first_ref_pressure_msg_received_setra = true;
            this->is_first_msg_ref_pressure_setra = false;
        }
    }
    void refPressureDPSMsgCallback(const rtf_sensors_msgs::msg::CustomPressureTemperature &pressure_msg)
    {
        this->lastRefPressureDPSMutex.lock();
        this->lastRefPressureMeasurementDPS = pressure_msg;
        this->lastRefPressureDPSMutex.unlock();
        if (this->is_first_msg_ref_pressure_dps)
        {
            this->first_ref_pressure_msg_received_dps = true;
            this->is_first_msg_ref_pressure_dps = false;
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
