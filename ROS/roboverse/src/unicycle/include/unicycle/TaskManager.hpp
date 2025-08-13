/*
    Task Manager - Header File

    This file contains the declaration of the TaskManager class, which is responsible for managing tasks on the unicycle.

    Written by: Ivan Sollazzo
*/

#ifndef TASK_MANAGER_HPP
#define TASK_MANAGER_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <vector>
#include <unicycle/srv/update_knowledge.hpp>

class TaskManager : public rclcpp::Node
{
public:
    // Constructor
    explicit TaskManager(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:

    // Unicycle's identifier
    std::string unicycle_id_;

    // Unicycle's number
    int unicycle_number_;

    // Number of unicycles
    int num_unicycles_;

    // Number of places
    int num_places_;

    // Vector of clients for knowledge saving
    std::vector<rclcpp::Client<unicycle::srv::UpdateKnowledge>::SharedPtr> knowledge_clients_;

    // Publisher for target pose
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr target_pose_pub_;

    // Subscriber for target reached
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr target_reached_sub_;

    // Subscriber for rendez-vous status
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr rendezvous_status_sub_;

    // Subscriber for Unity simulation status
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr unity_simulation_status_sub_;

    // Subscriber for temperature sensor
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr temperature_sensor_sub_;

    // Subscriber for humidity sensor
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr humidity_sensor_sub_;

    // Subscriber for air quality sensor
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr air_quality_sensor_sub_;

    // Publisher for rendez-vous desired pose
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr rendezvous_desired_pose_pub_;

    // Defining the FSM (Finite State Machine) in order to control the task manager
    enum class FSM
    {
        IDLE,
        GOING_TO_TARGET,
        COLLECTING_DATA,
        WAITING_FOR_RV,
        EXCHANGING_DATA,
        CHOOSING_NEXT_PLACE,
        FINALIZING
    };

    // Variable to hold the current state of the FSM
    FSM current_state_ = FSM::IDLE;

    // Defining some functions to handle the FSM states
    void handle_idle_state();
    void handle_going_to_target_state();
    void handle_collecting_data_state();
    void handle_waiting_for_rv_state();
    void handle_exchanging_data_state();
    void handle_choosing_next_place_state();
    void handle_finalizing_state();

    // Method to call the FSM
    void fsm();

    // Defining a method to handle the FSM transitions
    void transition_to_state(FSM new_state);

    // Callback for target reached
    void target_reached_callback(const std_msgs::msg::Bool::SharedPtr msg);

    // Callback for Unity simulation status
    void unity_simulation_status_callback(const std_msgs::msg::Bool::SharedPtr msg);

    // Callback for temperature sensor
    void temperature_sensor_callback(const std_msgs::msg::Float32::SharedPtr msg);

    // Callback for humidity sensor
    void humidity_sensor_callback(const std_msgs::msg::Float32::SharedPtr msg);

    // Callback for air quality sensor
    void air_quality_sensor_callback(const std_msgs::msg::Int32::SharedPtr msg);

    // Function to load places data
    void load_places_data();

    // Timer to periodically call the FSM
    rclcpp::TimerBase::SharedPtr fsm_timer_;

    // Variables to store poses (shared pointers)
    geometry_msgs::msg::Pose::SharedPtr target_place_0_;
    geometry_msgs::msg::Pose::SharedPtr target_place_1_;
    geometry_msgs::msg::Pose::SharedPtr target_place_2_;
    geometry_msgs::msg::Pose::SharedPtr rendezvous_desired_place_;

    // Variable to store current place
    int current_target_place_ = 0;

    // Variables to store current status
    bool target_reached_ = false;
    bool rendezvous_complete_ = false;
    bool unity_simulation_ready_ = false;
    bool rendezvous_failed_ = false;

    // Subscription array to track rendezvous status
    std::vector<bool> all_rendezvous_complete_;

    // Subscriptions array to track all rendezvous status
    std::vector<rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr> rendezvous_status_subs_;

    // Callback for rendezvous status according to the unicycle number
    void rendezvous_status_callback(const std_msgs::msg::Bool::SharedPtr msg, int unicycle_number);

    // Function to manage save knowledge requests
    void make_knowledge_update_request(const int unicycle_id);

    // Structure for data collection
    struct SensorsData {
        float temperature;
        float humidity;
        int air_quality;
    };

    // Data collection object
    SensorsData sensors_data_;

    // Variables to manage service requests
    bool service_request_sent_ = false;
    std::shared_future<std::shared_ptr<unicycle::srv::UpdateKnowledge::Response>> service_future_;

    // Add a timer for rendez-vous timeout
    rclcpp::TimerBase::SharedPtr rendezvous_timeout_timer_;

    // Callback to manage rendez-vous timeout
    void rendezvous_timeout_callback();

};

#endif // TASK_MANAGER_HPP