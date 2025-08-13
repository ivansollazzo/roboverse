/*
 Kinematics Controller - Header File
 This file contains the definition of the KinematicsController class,
 which is responsible for controlling the kinematics of the unicycle.
 Written by: Ivan Sollazzo
*/

#ifndef KINEMATICS_CONTROLLER_HPP
#define KINEMATICS_CONTROLLER_HPP

// Import necessary headers
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/bool.hpp>

// Kinematics controller class
class KinematicsController : public rclcpp::Node
{
public:
    // Constructor
    explicit KinematicsController(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
    // Unicycle's identifier
    std::string unicycle_id_;
    
    // Publisher for controller commands
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr ctrl_publisher_;

    // Publisher for target reached status
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr target_reached_publisher_;
    
    // Subscriber for current pose
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr current_pose_subscriber_;
    
    // Subscriber for target pose
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr target_pose_subscriber_;
    
    // Defining the FSM (Finite State Machine) in order to control the unicycle's movement
    enum class FSM
    {
        IDLE,       // The unicycle isn't doing anything
        ROTATING,   // The unicycle is rotating
        MOVING      // The unicycle is moving
    };
    
    // Variable to hold the current state of the FSM
    FSM current_state_ = FSM::IDLE;
    
    // Defining some methods to handle the FSM states
    void handle_idle_state();
    void handle_rotating_state();
    void handle_moving_state();

    // Method to call the FSM
    void fsm();
    
    // Defining a method to handle the FSM transitions
    void transition_to_state(FSM new_state);
    
    // Callback for current pose updates
    void current_pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg);
    
    // Callback for target pose updates
    void target_pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg);
    
    // Variables to store the current and target poses
    std::shared_ptr<geometry_msgs::msg::Pose> current_pose_;
    std::shared_ptr<geometry_msgs::msg::Pose> target_pose_;
    
    // Variables to store the control speeds
    std::shared_ptr<geometry_msgs::msg::Twist> control_speeds_;
    
    // Timer to periodically call the FSM
    rclcpp::TimerBase::SharedPtr fsm_timer_;
    
    // Method to calculate distance
    double calculate_distance(const geometry_msgs::msg::Pose &target_pose, const geometry_msgs::msg::Pose &current_pose);

    // Method to calculate angle
    double calculate_angle(const geometry_msgs::msg::Pose &target_pose, const geometry_msgs::msg::Pose &current_pose);

    // Boolean to check if the unicycle just started
    bool just_started_ = true;

    // Parameters
    double k_theta_;
    double k_beta_;
    double k_rho_;

    // Gains for rotation and translation
    double max_linear_speed_;
    double max_angular_speed_;
};
#endif // KINEMATICS_CONTROLLER_HPP