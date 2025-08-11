/*
    Task Manager - Source File

    This file contains the implementation of the TaskManager class, which is responsible for managing tasks on the unicycle.

    Written by: Ivan Sollazzo
*/

// Include the header file
#include "unicycle/TaskManager.hpp"

// Constructor
TaskManager::TaskManager(const rclcpp::NodeOptions &options)
    : Node("task_manager", options)
{
    // Get the unicycle's identifier. If not set, default to "unicycle_0"
    unicycle_id_ = this->declare_parameter<std::string>("unicycle_id", "unicycle_0");

    // Create a QoS profile with defaults settings for the publisher
    auto qos = rclcpp::SystemDefaultsQoS();

    // Initialize publisher for target pose
    target_pose_pub_ = this->create_publisher<geometry_msgs::msg::Pose>(unicycle_id_ + "/dynamics/target_pose", qos);

    // Initialize subscriber for target reached
    target_reached_sub_ = this->create_subscription<std_msgs::msg::Bool>(unicycle_id_ + "/dynamics/target_reached", qos, std::bind(&TaskManager::target_reached_callback, this, std::placeholders::_1));

    // Initialize subscriber for rendez-vous status
    rendezvous_status_sub_ = this->create_subscription<std_msgs::msg::Bool>(unicycle_id_ + "/rendezvous/complete", qos, std::bind(&TaskManager::rendezvous_status_callback, this, std::placeholders::_1));

    // Initialize subscriber for Unity simulation status
    unity_simulation_status_sub_ = this->create_subscription<std_msgs::msg::Bool>("simulation/ready", qos, std::bind(&TaskManager::unity_simulation_status_callback, this, std::placeholders::_1));

    // Initialize publisher for rendez-vous desired pose
    rendezvous_desired_pose_pub_ = this->create_publisher<geometry_msgs::msg::Pose>(unicycle_id_ + "/rendezvous/desired_pose", qos);

    // Initialize a timer that calls the FSM at a regular interval
    fsm_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&TaskManager::fsm, this));

    // Variable to set the current target place according to unicycle id
    current_target_place_ = (unicycle_id_ == "unicycle_1") ? 1 : (unicycle_id_ == "unicycle_2") ? 2 : 0;

    // Load places data
    load_places_data();

}

// Method to call the FSM
void TaskManager::fsm()
{
    switch (current_state_) {
        case FSM::IDLE:
            handle_idle_state();
            break;
        case FSM::GOING_TO_TARGET:
            handle_going_to_target_state();
            break;
        case FSM::COLLECTING_DATA:
            handle_collecting_data_state();
            break;
        case FSM::WAITING_FOR_RV:
            handle_waiting_for_rv_state();
            break;
        case FSM::EXCHANGING_DATA:
            handle_exchanging_data_state();
            break;
        case FSM::FINALIZING:
            handle_finalizing_state();
            break;
    }
}

// Function to handle state transitions
void TaskManager::transition_to_state(FSM new_state)
{
    current_state_ = new_state;
}

// Function to handle IDLE state
void TaskManager::handle_idle_state()
{
    // If simulation is not ready return
    if (!unity_simulation_ready_)
    {
        return;
    }

    // Publish the target pose based on the current target place
    switch (current_target_place_)
    {
    case 0:
        target_pose_pub_->publish(*target_place_0_);
        break;
    case 1:
        target_pose_pub_->publish(*target_place_1_);
        break;
    case 2:
        target_pose_pub_->publish(*target_place_2_);
        break;
    default:
        RCLCPP_WARN(this->get_logger(), "Unknown target place");
        break;
    }

    // Transition to the GOING_TO_TARGET state
    transition_to_state(FSM::GOING_TO_TARGET);
}

// Function to handle the GOING_TO_TARGET_STATE
void TaskManager::handle_going_to_target_state()
{
    // Wait for the target pose to be reached
    if (target_reached_)
    {
        // Transition to the COLLECTING_DATA state
        transition_to_state(FSM::IDLE);
    }
}

void TaskManager::handle_collecting_data_state()
{
    return;
}

// Function to handle the WAITING_FOR_RV state
void TaskManager::handle_waiting_for_rv_state()
{
    return;
}

// Function to handle the EXCHANGING_DATA state
void TaskManager::handle_exchanging_data_state()
{
    return;
}

// Function to handle the FINALIZING state
void TaskManager::handle_finalizing_state()
{
    return;
}

// Callback to handle target reached
void TaskManager::target_reached_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    target_reached_ = msg->data;
}

// Callback to handle rendez-vous status
void TaskManager::rendezvous_status_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    rendezvous_complete_ = msg->data;
}

void TaskManager::load_places_data() {
    
    // Instantiate places objects
    target_place_0_ = std::make_shared<geometry_msgs::msg::Pose>();
    target_place_1_ = std::make_shared<geometry_msgs::msg::Pose>();
    target_place_2_ = std::make_shared<geometry_msgs::msg::Pose>();
    rendezvous_desired_place_ = std::make_shared<geometry_msgs::msg::Pose>();

    // Load from parameters
    target_place_0_->position.x = this->declare_parameter("target_place_0_x", 0.0);
    target_place_0_->position.z = this->declare_parameter("target_place_0_z", 0.0);

    target_place_1_->position.x = this->declare_parameter("target_place_1_x", 0.0);
    target_place_1_->position.z = this->declare_parameter("target_place_1_z", 0.0);

    target_place_2_->position.x = this->declare_parameter("target_place_2_x", 0.0);
    target_place_2_->position.z = this->declare_parameter("target_place_2_z", 0.0);

    rendezvous_desired_place_->position.x = this->declare_parameter("rv_desired_place_x", 0.0);
    rendezvous_desired_place_->position.z = this->declare_parameter("rv_desired_place_z", 0.0);

    // Print loaded places data
    RCLCPP_INFO(this->get_logger(), "Target place 0: [%.2f, %.2f]",
                target_place_0_->position.z, target_place_0_->position.x);
    RCLCPP_INFO(this->get_logger(), "Target place 1: [%.2f, %.2f]",
                target_place_1_->position.z, target_place_1_->position.x);
    RCLCPP_INFO(this->get_logger(), "Target place 2: [%.2f, %.2f]",
                target_place_2_->position.z, target_place_2_->position.x);
    RCLCPP_INFO(this->get_logger(), "Rendez-vous desired place: [%.2f, %.2f]",
                rendezvous_desired_place_->position.x, rendezvous_desired_place_->position.x);

}

void TaskManager::unity_simulation_status_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    unity_simulation_ready_ = msg->data;
}

// Main function
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TaskManager>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}