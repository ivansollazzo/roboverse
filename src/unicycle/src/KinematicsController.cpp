/*
    Kinematics Controller - Source File

    This file contains the implementation of the KinematicsController class,
    which is responsible for controlling the kinematics of the unicycle.

    Written by: Ivan Sollazzo
*/

// Include the header file
#include "unicycle/KinematicsController.hpp"

// Constructor
KinematicsController::KinematicsController(const rclcpp::NodeOptions &options)
    : Node("kinematics_controller", options)
{
    // Get the unicycle's identifier. If not set, default to "unicycle_0"
    unicycle_id_ = this->declare_parameter<std::string>("unicycle_id", "unicycle_0");

    // Set the max speed parameters
    max_linear_speed_ = this->declare_parameter<double>("max_linear_speed", 1.0);
    max_angular_speed_ = this->declare_parameter<double>("max_angular_speed", 1.0);

    // Set the gains of rotation and translation
    k_theta_ = this->declare_parameter<double>("k_theta", 1.0);
    k_beta_ = this->declare_parameter<double>("k_beta", 1.0);
    k_rho_ = this->declare_parameter<double>("k_rho", 1.0);

    // Create a QoS profile with custom settings for all nodes except current_pose
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
    .reliable()
    .transient_local();

    // Create a QoS profile with default settings for current_pose
    auto unity_qos = rclcpp::SystemDefaultsQoS();

    // Initialize publisher for control commands
    ctrl_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(unicycle_id_ + "/dynamics/ctrl", unity_qos);

    // Initialize subscriber for current pose
    current_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::Pose>(unicycle_id_ + "/dynamics/current_pose", unity_qos, std::bind(&KinematicsController::current_pose_callback, this, std::placeholders::_1));

    // Initialize publisher for target reached
    target_reached_publisher_ = this->create_publisher<std_msgs::msg::Bool>(unicycle_id_ + "/dynamics/target_reached", qos);

    // Initialize subscriber for target pose
    target_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::Pose>(unicycle_id_ + "/dynamics/target_pose", qos, std::bind(&KinematicsController::target_pose_callback, this, std::placeholders::_1));

    // Initialize a timer that calls the FSM at a regular interval
    fsm_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&KinematicsController::fsm, this));

    // Initialize control speeds
    control_speeds_ = std::make_shared<geometry_msgs::msg::Twist>();

    // Initialize poses
    current_pose_ = std::make_shared<geometry_msgs::msg::Pose>();
    target_pose_ = std::make_shared<geometry_msgs::msg::Pose>();

}

// Callback for current pose updates
void KinematicsController::current_pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
{
    if (*msg != *current_pose_)
    {
        // Store the current pose
        *current_pose_ = *msg;

        // If just started, set target pose to current pose
        if (just_started_) 
        {
            *target_pose_ = *current_pose_;
            just_started_ = false;
        }
    }
}

// Callback for target pose updates
void KinematicsController::target_pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
{
    // Store the target pose
    *target_pose_ = *msg;
}

// Function to call the FSM
void KinematicsController::fsm()
{
    switch (current_state_)
    {
    case FSM::IDLE:
        handle_idle_state();
        break;
    case FSM::ROTATING:
        handle_rotating_state();
        break;
    case FSM::MOVING:
        handle_moving_state();
        break;
    }
}

// Function to handle state transitions
void KinematicsController::transition_to_state(FSM new_state)
{
    current_state_ = new_state;
}

// Function to handle idle state
void KinematicsController::handle_idle_state()
{
    // We don't even calculate anything if current pose and target pose are equal. But sometimes there might be some errors due to approximations.
    if (*target_pose_ == *current_pose_)
    {
        return;
    }

    // In order to make sure that a new target pose is valid, we calculate the difference between the current and target poses in terms of distance and angle
    bool valid_target_ = false;

    // Calculate the distance
    if (calculate_distance(*target_pose_, *current_pose_) > 0.1)
    {
        valid_target_ = true;
    }
    else if (calculate_angle(*target_pose_, *current_pose_) > 0.1)
    {
        valid_target_ = true;
    }

    // If the target is valid, we transition to the rotating state. Instead, we keep the current state and also update the target pose to the current pose in order to avoid unuseful computations.
    if (valid_target_)
    {
        RCLCPP_INFO(this->get_logger(), "Valid target pose received -> Z: %f, X: %f, Orientation: %f", target_pose_->position.z, target_pose_->position.x, target_pose_->orientation.y);

        // Publish target reached
        auto target_reached_msg = std::make_shared<std_msgs::msg::Bool>();
        target_reached_msg->data = false;

        target_reached_publisher_->publish(*target_reached_msg);

        transition_to_state(FSM::ROTATING);
    }
}

// Function to handle rotating state
void KinematicsController::handle_rotating_state()
{
    // We calculate the angle error
    double angle_error_ = calculate_angle(*target_pose_, *current_pose_);

    // If the angle error is small enough, we can switch to the moving state
    if (std::abs(angle_error_) < 0.1)
    {
        // Update control speeds
        control_speeds_->angular.y = 0.0;
        control_speeds_->linear.z = 0.0;

        // Publish the control speeds
        ctrl_publisher_->publish(*control_speeds_);

        transition_to_state(FSM::MOVING);
    }
    else
    {
        // Update the control speeds according to the control law
        control_speeds_->linear.z = 0.0;
        control_speeds_->angular.y = std::min(-k_theta_ * angle_error_, max_angular_speed_);

        // Publish the control speeds
        ctrl_publisher_->publish(*control_speeds_);

        RCLCPP_INFO(this->get_logger(), "Currently rotating. Angle error: %f. Linear speed: %f. Angular speed: %f.", angle_error_, control_speeds_->linear.z, control_speeds_->angular.y);
    }
}

// Function to handle moving state
void KinematicsController::handle_moving_state()
{
    // We calculate the distance error
    double distance_error_ = calculate_distance(*target_pose_, *current_pose_);
    double angle_error_ = calculate_angle(*target_pose_, *current_pose_);

    // If the distance error is small enough, we can switch to the idle state
    if (distance_error_ < 0.1)
    {
        // Update control speeds
        control_speeds_->linear.z = 0.0;
        control_speeds_->angular.y = 0.0;

        // Publish the control speeds
        ctrl_publisher_->publish(*control_speeds_);

        // Log the updated current pose
        RCLCPP_INFO(this->get_logger(), "Current pose updated to target pose -> Z: %f, X: %f, Orientation: %f", current_pose_->position.z, current_pose_->position.x, current_pose_->orientation.y);

        // Publish target reached
        auto target_reached_msg = std::make_shared<std_msgs::msg::Bool>();
        target_reached_msg->data = true;

        target_reached_publisher_->publish(*target_reached_msg);

        // Switch to idle state
        transition_to_state(FSM::IDLE);
    }
    else
    {
        // Update the control speeds according to the control law.
        control_speeds_->linear.z = std::min(k_rho_ * distance_error_, max_linear_speed_);
        control_speeds_->angular.y = std::min(-k_beta_ * angle_error_, max_angular_speed_);

        // Publish the control speeds
        ctrl_publisher_->publish(*control_speeds_);

        RCLCPP_INFO(this->get_logger(), "Currently moving. Distance error: %f, Angle error: %f. Linear speed: %f. Angular speed: %f.", distance_error_, angle_error_, control_speeds_->linear.z, control_speeds_->angular.y);
    }

}

// Method to calculate distance
double KinematicsController::calculate_distance(const geometry_msgs::msg::Pose &target_pose, const geometry_msgs::msg::Pose &current_pose)
{
    // Calculate the distance
    double dz = current_pose.position.z - target_pose.position.z;
    double dx = current_pose.position.x - target_pose.position.x;
    
    return std::hypot(dz, dx);
}

// Method to calculate angle
double KinematicsController::calculate_angle(const geometry_msgs::msg::Pose &target_pose, const geometry_msgs::msg::Pose &current_pose)
{
    // Get current orientation angle
    double current_theta = current_pose.orientation.y;

    // Calculate the desired angle
    double target_theta = std::atan2(
        target_pose.position.x - current_pose.position.x, 
        target_pose.position.z - current_pose.position.z
    );
    
    // Calculate angle error
    double angle_error = current_theta - target_theta;

    // Normalize the angle error to the range [-pi, pi]
    angle_error = std::atan2(std::sin(angle_error), std::cos(angle_error));

    return angle_error;
}

// Main function
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KinematicsController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}