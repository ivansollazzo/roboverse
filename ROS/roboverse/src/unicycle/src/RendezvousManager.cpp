/*
    Rendez-vous Manager - Source File

    This file contains the implementation of the RendezvousManager class,
    which is responsible for managing rendezvous points between different
    agents in the system.

    Written by: Ivan Sollazzo
*/

// Include the header file only
#include "unicycle/RendezvousManager.hpp"

// Constructor
RendezvousManager::RendezvousManager(const rclcpp::NodeOptions &options)
    : Node("rendezvous_manager", options)
{
    // Get the unicycle's identifier. If not set, default to "unicycle_0"
    unicycle_id_ = this->declare_parameter<std::string>("unicycle_id", "unicycle_0");

    // Get the unicycle number
    unicycle_number_ = this->declare_parameter<int>("unicycle_number", 0);

    // Create a QoS profile with custom settings for all nodes except current_pose
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
    .reliable()
    .transient_local();

    // Initialize an array that tracks if all the robots have received their desired poses
    int num_unicycles = this->declare_parameter<int>("num_unicycles", 1);
    desired_poses_received_.resize(num_unicycles, false);

    // Instantiate the desired pose subs
    desired_poses_subs_.reserve(num_unicycles);

    // Initialize subscriptions for desired poses
    for (int i = 0; i < num_unicycles; ++i) {
        std::string topic_name = "unicycle_" + std::to_string(i) + "/rendezvous/desired_pose";
        desired_poses_subs_.emplace_back(this->create_subscription<geometry_msgs::msg::Pose>(
            topic_name, qos,
            [this, i](const geometry_msgs::msg::Pose::SharedPtr msg) {
                this->desired_pose_received_callback(msg, i);
            }
        ));
    }

    // Initialize array of desired poses
    desired_poses_.resize(num_unicycles);

    // Initialize publisher for target pose
    target_pose_pub_ = this->create_publisher<geometry_msgs::msg::Pose>(
        unicycle_id_ + "/dynamics/target_pose", qos
    );

    // Initialize publisher for rendez-vous status
    rendezvous_status_pub_ = this->create_publisher<std_msgs::msg::Bool>(
        unicycle_id_ + "/rendezvous/complete", qos
    );

    // Initialize subscription for target reached
    target_reached_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        unicycle_id_ + "/dynamics/target_reached", qos,
        std::bind(&RendezvousManager::target_reached_callback, this, std::placeholders::_1)
    );

    // Initialize a timer to periodically call the FSM
    fsm_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&RendezvousManager::fsm, this));
}

// Callback for desired pose subscription
void RendezvousManager::desired_pose_received_callback(const geometry_msgs::msg::Pose::SharedPtr msg, const int unicycle_id)
{
    // Update the desired pose for the corresponding unicycle
    desired_poses_[unicycle_id] = msg;

    // Mark the desired pose as received
    desired_poses_received_[unicycle_id] = true;

    // Log the received desired pose
    if (unicycle_id == unicycle_number_) {
        RCLCPP_INFO(this->get_logger(), "Set desired pose to: [%f, %f]", msg->position.z, msg->position.x);
    }
    else {
        RCLCPP_INFO(this->get_logger(), "Received desired pose from unicycle by id %d: [%f, %f]", unicycle_id, msg->position.z, msg->position.x);
    }
}

// Function to call the FSM
void RendezvousManager::fsm() {
    switch (current_state_) {
        case FSM::IDLE:
            handle_idle_state();
            break;
        case FSM::WAITING_FOR_DESIRED_PLACES:
            handle_waiting_for_desired_places_state();
            break;
        case FSM::CALCULATING_RV_PLACE:
            handle_calculating_rv_place_state();
            break;
        case FSM::GOING_TO_RV_PLACE:
            handle_going_to_rv_place_state();
            break;
        case FSM::FINALIZING:
            handle_finalizing_state();
            break;
    }
}

// Function to handle idle state
void RendezvousManager::handle_idle_state() {

    // If the desired place is received from taskmanager, transition to the WAITING_FOR_DESIRED_PLACES state
    if (desired_poses_received_[unicycle_number_]) {
        transition_to_state(FSM::WAITING_FOR_DESIRED_PLACES);
    }
}

// Function to handle waiting for desired places state
void RendezvousManager::handle_waiting_for_desired_places_state() {
    
    bool all_desired_poses_received = true;

    // Check if all unicycles have published their desired poses
    for (const auto &received : desired_poses_received_) {
        if (!received) {
            all_desired_poses_received = false;
            break;
        }
    }

    if (all_desired_poses_received) {
        
        RCLCPP_INFO(this->get_logger(), "All desired poses received. Calculating rendez-vous position...");
        
        transition_to_state(FSM::CALCULATING_RV_PLACE);
    
    }
}

// Function to handle calculating rendezvous place state
void RendezvousManager::handle_calculating_rv_place_state() {

    // Calculate RV place. At this time we just publish the target pose
    target_pose_pub_->publish(*desired_poses_[unicycle_number_]);

    // Switch to GOING_TO_RV_PLACE state
    transition_to_state(FSM::GOING_TO_RV_PLACE);
}

// Function to handle going to rendezvous place state
void RendezvousManager::handle_going_to_rv_place_state() {
    
    if (!target_reached_) {
        return;
    }

    // Switch to FINALIZING state
    transition_to_state(FSM::FINALIZING);
}

// Function to handle finalizing state
void RendezvousManager::handle_finalizing_state() {

    // Publish rendezvous status
    std_msgs::msg::Bool::SharedPtr status_msg = std::make_shared<std_msgs::msg::Bool>();
    status_msg->data = true;
    rendezvous_status_pub_->publish(*status_msg);

    // Cleaning variables
    target_reached_ = false;

    // Setting all desired poses received flags to false
    desired_poses_received_.assign(desired_poses_received_.size(), false);

    // Resetting the desired poses
    for (auto &pose : desired_poses_) {
        pose.reset();
    }

    // Switch to IDLE state
    transition_to_state(FSM::IDLE);

}

// Function to handle transition
void RendezvousManager::transition_to_state(FSM new_state) {
    current_state_ = new_state;
}

// Callback to check if target is reached
void RendezvousManager::target_reached_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    target_reached_ = msg->data;
}

// Main function
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RendezvousManager>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}