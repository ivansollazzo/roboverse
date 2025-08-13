/*
    Sensors Manager - Header File

    This file contains the declaration of the SensorsManager class, which is responsible for managing sensor data on the unicycle.

    Written by: Ivan Sollazzo
*/

#ifndef SENSORS_MANAGER_HPP
#define SENSORS_MANAGER_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>

class SensorsManager : public rclcpp::Node
{
public:
    explicit SensorsManager(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
    // Unicycle ID
    std::string unicycle_id_;

    int unicycle_number_;

    // Subscriber for raw data
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr raw_data_sub_;

    // Unicycle sensor data
    float current_temperature_ = 0.0;
    float current_humidity_ = 0.0;
    int current_air_quality_ = 0.0;

    // Publishers for processed data
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr temperature_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr humidity_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr air_quality_pub_;

    // Callback for raw data
    void raw_data_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

    // Timer to publish sensor data
    rclcpp::TimerBase::SharedPtr timer_;

    // Function to publish sensor data
    void publish_sensor_data();
};

#endif // SENSORS_MANAGER_HPP