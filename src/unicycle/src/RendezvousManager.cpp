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

    // Set the parameters to configure the rendez-vous behaviors
    rendezvous_error_update_factor_ = this->declare_parameter<double>("rendezvous_error_update_factor", 0.25);

    rendezvous_error_threshold_ = this->declare_parameter<double>("rendezvous_error_threshold", 1.5);

    // Create a QoS profile with custom settings for all nodes except current_pose
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
                   .reliable()
                   .transient_local();

    // Create a QoS profile with default settings for current_pose
    auto unity_qos = rclcpp::SystemDefaultsQoS();

    // Initialize an array that tracks if all the robots have received their desired poses
    num_unicycles_ = this->declare_parameter<int>("num_unicycles", 1);
    desired_poses_received_.resize(num_unicycles_, false);

    // Instantiate the desired pose subs
    desired_poses_subs_.reserve(num_unicycles_);

    // Instantiate the current poses subs
    current_poses_subs_.reserve(num_unicycles_);

    // Initialize an array to hold the current poses of all unicycles
    current_poses_.resize(num_unicycles_);

    // Initialize subscriptions for desired poses
    for (int i = 0; i < num_unicycles_; ++i)
    {
        std::string topic_name = "unicycle_" + std::to_string(i) + "/rendezvous/desired_pose";
        desired_poses_subs_.emplace_back(this->create_subscription<geometry_msgs::msg::Pose>(
            topic_name, qos,
            [this, i](const geometry_msgs::msg::Pose::SharedPtr msg)
            {
                this->desired_pose_received_callback(msg, i);
            }));
    }

    // Initialize subscriptions for current poses
    for (int i = 0; i < num_unicycles_; ++i)
    {
        std::string topic_name = "unicycle_" + std::to_string(i) + "/dynamics/current_pose";
        current_poses_subs_.emplace_back(this->create_subscription<geometry_msgs::msg::Pose>(
            topic_name, unity_qos,
            [this, i](const geometry_msgs::msg::Pose::SharedPtr msg)
            {
                this->current_pose_received_callback(msg, i);
            }));
    }

    // Initialize array of desired poses
    desired_poses_.resize(num_unicycles_);

    // Initialize publisher for target pose
    target_pose_pub_ = this->create_publisher<geometry_msgs::msg::Pose>(
        unicycle_id_ + "/dynamics/target_pose", qos);

    // Initialize publisher for rendez-vous status
    rendezvous_status_pub_ = this->create_publisher<std_msgs::msg::Bool>(
        unicycle_id_ + "/rendezvous/complete", qos);

    // Initialize subscription for target reached
    target_reached_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        unicycle_id_ + "/dynamics/target_reached", qos,
        std::bind(&RendezvousManager::target_reached_callback, this, std::placeholders::_1));

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
    if (unicycle_id == unicycle_number_)
    {
        RCLCPP_INFO(this->get_logger(), "Set desired pose to: [%f, %f]", msg->position.z, msg->position.x);
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Received desired pose from unicycle by id %d: [%f, %f]", unicycle_id, msg->position.z, msg->position.x);
    }
}

// Callback for current pose subscription
void RendezvousManager::current_pose_received_callback(const geometry_msgs::msg::Pose::SharedPtr msg, const int unicycle_id)
{
    // Update the current pose for the corresponding unicycle
    current_poses_[unicycle_id] = msg;
}

// Function to call the FSM
void RendezvousManager::fsm()
{
    switch (current_state_)
    {
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
void RendezvousManager::handle_idle_state()
{

    // If the desired place is received from taskmanager, transition to the WAITING_FOR_DESIRED_PLACES state
    if (desired_poses_received_[unicycle_number_])
    {
        transition_to_state(FSM::WAITING_FOR_DESIRED_PLACES);
    }
}

// Function to handle waiting for desired places state
void RendezvousManager::handle_waiting_for_desired_places_state()
{

    bool all_desired_poses_received = true;

    // Check if all unicycles have published their desired poses
    for (const auto &received : desired_poses_received_)
    {
        if (!received)
        {
            all_desired_poses_received = false;
            break;
        }
    }

    if (all_desired_poses_received)
    {

        RCLCPP_INFO(this->get_logger(), "All desired poses received. Calculating rendez-vous position...");

        transition_to_state(FSM::CALCULATING_RV_PLACE);
    }
}

// Function to handle calculating rendezvous place state
void RendezvousManager::handle_calculating_rv_place_state()
{
    // Now we calculate the error between the current position of this unicycle and its desired pose
    double qz = current_poses_[unicycle_number_]->position.z - desired_poses_[unicycle_number_]->position.z;
    
    double qx = current_poses_[unicycle_number_]->position.x - desired_poses_[unicycle_number_]->position.x;

    // Now we have to create an array that contains the differences between the previously calculated error and the errors of the other unicycles

    // We create a double that contains the error
    double u_z = 0.0;
    double u_x = 0.0;

    for (int i = 0; i < num_unicycles_; ++i)
    {
        if (i != unicycle_number_)
        {
            // Calculate the error of the other unicycle
            double dz_i = current_poses_[i]->position.z - desired_poses_[i]->position.z;
            double dx_i = current_poses_[i]->position.x - desired_poses_[i]->position.x;

            // Calculate the error between this unicycle and the other
            double dz = qz - dz_i;
            double dx = qx - dx_i;

            // Add to the sum
            u_z += dz;
            u_x += dx;
        }
    }

    // Store the rendezvous errors. We need them in order to decide the next state after the unicycles reaches the target pose.
    rendezvous_error_x_ = u_x;
    rendezvous_error_z_ = u_z;

    // Now we multiply the results by the error update factor
    u_z *= rendezvous_error_update_factor_;
    u_x *= rendezvous_error_update_factor_;

    // So there's a new target point to reach
    geometry_msgs::msg::Pose::SharedPtr target_pose = std::make_shared<geometry_msgs::msg::Pose>();
    target_pose->position.z = current_poses_[unicycle_number_]->position.z - u_z;
    target_pose->position.x = current_poses_[unicycle_number_]->position.x - u_x;

    // Publish the target pose
    target_pose_pub_->publish(*target_pose);

    // Switch to GOING_TO_RV_PLACE state
    transition_to_state(FSM::GOING_TO_RV_PLACE);
}

// Function to handle going to rendezvous place state
void RendezvousManager::handle_going_to_rv_place_state()
{

    if (!target_reached_)
    {
        return;
    }

    if (rendezvous_error_z_ < rendezvous_error_threshold_ && rendezvous_error_x_ < rendezvous_error_threshold_)
    {
        transition_to_state(FSM::FINALIZING);
    }
    else {
        transition_to_state(FSM::CALCULATING_RV_PLACE);
    }
}

// Function to handle finalizing state
void RendezvousManager::handle_finalizing_state()
{

    // Publish rendezvous status
    std_msgs::msg::Bool::SharedPtr status_msg = std::make_shared<std_msgs::msg::Bool>();
    status_msg->data = true;
    rendezvous_status_pub_->publish(*status_msg);

    // Cleaning variables
    target_reached_ = false;
    rendezvous_error_x_ = 0.0;
    rendezvous_error_z_ = 0.0;

    // Setting all desired poses received flags to false
    desired_poses_received_.assign(desired_poses_received_.size(), false);

    // Resetting the desired poses
    for (auto &pose : desired_poses_)
    {
        pose.reset();
    }

    // Switch to IDLE state
    transition_to_state(FSM::IDLE);
}

// Function to handle transition
void RendezvousManager::transition_to_state(FSM new_state)
{
    current_state_ = new_state;
}

// Callback to check if target is reached
void RendezvousManager::target_reached_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    target_reached_ = msg->data;
}

// Main function
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RendezvousManager>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}