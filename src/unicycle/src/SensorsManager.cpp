/*
    Sensors Manager - Source File

    This file contains the implementation of the SensorsManager class, which is responsible for managing sensor data on the unicycle.

    Written by: Ivan Sollazzo
*/

// Include the header file only
#include "unicycle/SensorsManager.hpp"

// Constructor
SensorsManager::SensorsManager(const rclcpp::NodeOptions &options)
    : Node("sensors_manager", options)
{
    // Get the unicycle's identifier. If not set, default to "unicycle_0"
    unicycle_id_ = this->declare_parameter<std::string>("unicycle_id", "unicycle_0");

    // Get the unicycle number
    unicycle_number_ = this->declare_parameter<int>("unicycle_number", 0);

    // Create a QoS profile with custom settings for all nodes only
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
                   .reliable()
                   .transient_local();

    // Create a QoS profile with default settings for Unity only
    auto unity_qos = rclcpp::SystemDefaultsQoS();

    // Initialize subscriber for sensor raw data
    raw_data_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "simulation/raw_data_" + std::to_string(unicycle_number_), unity_qos,
        std::bind(&SensorsManager::raw_data_callback, this, std::placeholders::_1));

    // Initialize publishers for processed data
    temperature_pub_ = this->create_publisher<std_msgs::msg::Float32>(unicycle_id_ + "/sensors/temperature", qos);
    humidity_pub_ = this->create_publisher<std_msgs::msg::Float32>(unicycle_id_ + "/sensors/humidity", qos);
    air_quality_pub_ = this->create_publisher<std_msgs::msg::Int32>(unicycle_id_ + "/sensors/air_quality", qos);

    // Start a timer to publish sensor data at a regular interval
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1), std::bind(&SensorsManager::publish_sensor_data, this));
}

// Callback for raw data
void SensorsManager::raw_data_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    // Update sensor data with the received message
    if (msg->data.size() >= 3)
    {
        current_temperature_ = msg->data[0];
        current_humidity_ = msg->data[1];
        current_air_quality_ = static_cast<int>(msg->data[2]);
    }
}

// Function to publish sensor data
void SensorsManager::publish_sensor_data()
{
    // Create messages for each sensor data type
    auto temperature_msg = std_msgs::msg::Float32();
    temperature_msg.data = current_temperature_;

    auto humidity_msg = std_msgs::msg::Float32();
    humidity_msg.data = current_humidity_;

    auto air_quality_msg = std_msgs::msg::Int32();
    air_quality_msg.data = current_air_quality_;

    temperature_pub_->publish(temperature_msg);
    humidity_pub_->publish(humidity_msg);
    air_quality_pub_->publish(air_quality_msg);
}

// Main function
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SensorsManager>());
    rclcpp::shutdown();
    return 0;
}