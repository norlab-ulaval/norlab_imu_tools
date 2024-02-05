#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
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
        sensorPressureIn = this->create_subscription<sensor_msgs::msg::FluidPressure>("sensor_pressure_in", 10,
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

    double initial_altitude = 0.0;

    std::string formula;
    sensor_msgs::msg::Temperature lastRefTempMeasurement;
    std::mutex lastRefTempMutex;
    sensor_msgs::msg::FluidPressure lastRefPressureMeasurement;
    std::mutex lastRefPressureMutex;

    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr altitudePub;
    rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr refPressureIn;
    rclcpp::Subscription<sensor_msgs::msg::Temperature>::SharedPtr refTempIn;
    rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr sensorPressureIn;

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
            this->lastRefPressureMutex.lock();
            double exponent_part = std::pow(P/this->lastRefPressureMeasurement.fluid_pressure, (this->R*this->Lb)/(this->g*this->M));
            this->lastRefPressureMutex.unlock();
            this->lastRefTempMutex.lock();
            altitude = this->hb - ((this->lastRefTempMeasurement.temperature/this->Lb)*(exponent_part - 1));
            this->lastRefTempMutex.unlock();
        }
        else if (this->formula == "hypsometric")
        {
            // assuming the virtual temperature is the temperature measured by the dps sensor.
            double exponent_part = std::pow(P/this->Pb, (this->R*this->Lb)/(this->g*this->M));
            altitude = this->hb - ((this->Tb/this->Lb)*(exponent_part - 1));
        }
        else
        {
            this->lastRefTempMutex.lock();
            altitude = ((this->R * (this->lastRefTempMeasurement.temperature + 273.15))/this->g) * std::log(this->P0/P);
            this->lastRefTempMutex.unlock();
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
    void refTempMsgCallback(const sensor_msgs::msg::Temperature &temp_msg)
    {
        this->lastRefTempMutex.lock();
        this->lastRefTempMeasurement = temp_msg;
        this->lastRefTempMutex.unlock();
    }
    void refPressureMsgCallback(const sensor_msgs::msg::FluidPressure &pressure_msg)
    {
        this->lastRefPressureMutex.lock();
        this->lastRefPressureMeasurement = pressure_msg;
        this->lastRefPressureMutex.unlock();
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<altitudeComputation>());
    rclcpp::shutdown();
    return 0;
}
