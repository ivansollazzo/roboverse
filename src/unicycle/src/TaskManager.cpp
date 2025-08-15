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

    // Get the unicycle's number
    unicycle_number_ = this->declare_parameter<int>("unicycle_number", 0);

    // Get the number of unicycles
    num_unicycles_ = this->declare_parameter<int>("num_unicycles", 1);

    // Get the number of places
    num_places_ = this->declare_parameter<int>("num_places", 1);

    // Create a QoS profile with custom settings for all nodes except current_pose
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
                   .reliable()
                   .transient_local();

    // Create a QoS profile with default settings for current_pose
    auto unity_qos = rclcpp::SystemDefaultsQoS();

    // Initialize publisher for target pose
    target_pose_pub_ = this->create_publisher<geometry_msgs::msg::Pose>(unicycle_id_ + "/dynamics/target_pose", qos);

    // Initialize subscriber for target reached
    target_reached_sub_ = this->create_subscription<std_msgs::msg::Bool>(unicycle_id_ + "/dynamics/target_reached", qos, std::bind(&TaskManager::target_reached_callback, this, std::placeholders::_1));

    // Initialize subscriber for Unity simulation status
    unity_simulation_status_sub_ = this->create_subscription<std_msgs::msg::Bool>("simulation/ready", unity_qos, std::bind(&TaskManager::unity_simulation_status_callback, this, std::placeholders::_1));

    // Instantiate the rendezvous status subs
    rendezvous_status_subs_.resize(num_unicycles_);

    // Instantiate the rendezvous complete booleans
    all_rendezvous_complete_.resize(num_unicycles_);

    // Initialize subscriptions for all unicycles rendezvous status
    for (int i = 0; i < num_unicycles_; ++i)
    {
        std::string topic_name = "unicycle_" + std::to_string(i) + "/rendezvous/complete";
        rendezvous_status_subs_[i] = this->create_subscription<std_msgs::msg::Bool>(
            topic_name, qos,
            [this, i](const std_msgs::msg::Bool::SharedPtr msg)
            {
                this->rendezvous_status_callback(msg, i);
            });
    }

    // Initialize publisher for rendez-vous desired pose
    rendezvous_desired_pose_pub_ = this->create_publisher<geometry_msgs::msg::Pose>(unicycle_id_ + "/rendezvous/desired_pose", qos);

    // Initialize a timer that calls the FSM at a regular interval
    fsm_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&TaskManager::fsm, this));

    // Variable to set the current target place according to unicycle id
    current_target_place_ = this->declare_parameter<int>("current_target_place", 0);

    // Load places data
    load_places_data();

    // Initialize clients for knowledge saving
    knowledge_clients_.resize(num_unicycles_);
    for (int i = 0; i < num_unicycles_; ++i)
    {
        knowledge_clients_[i] = this->create_client<unicycle::srv::UpdateKnowledge>("unicycle_" + std::to_string(i) + "/knowledge/update_knowledge");
    }

    // Initialize subscriptions for sensor data
    temperature_sensor_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        unicycle_id_ + "/sensors/temperature", qos,
        std::bind(&TaskManager::temperature_sensor_callback, this, std::placeholders::_1));

    humidity_sensor_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        unicycle_id_ + "/sensors/humidity", qos,
        std::bind(&TaskManager::humidity_sensor_callback, this, std::placeholders::_1));

    air_quality_sensor_sub_ = this->create_subscription<std_msgs::msg::Int32>(
        unicycle_id_ + "/sensors/air_quality", qos,
        std::bind(&TaskManager::air_quality_sensor_callback, this, std::placeholders::_1));

    // Initialize a timer for rendez-vous timeout (disabled by default)
    rendezvous_timeout_timer_ = this->create_wall_timer(std::chrono::seconds(90), std::bind(&TaskManager::rendezvous_timeout_callback, this));
    rendezvous_timeout_timer_->cancel();
}

// Method to call the FSM
void TaskManager::fsm()
{
    switch (current_state_)
    {
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
    case FSM::CHOOSING_NEXT_PLACE:
        handle_choosing_next_place_state();
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

    // Publish the target pose according to the current target place
    target_pose_pub_->publish(*target_places_[current_target_place_]);

    RCLCPP_INFO(this->get_logger(), "This unicycle will visit target place %d as first.", current_target_place_);

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
        transition_to_state(FSM::COLLECTING_DATA);
    }
}

void TaskManager::handle_collecting_data_state()
{
    // Make knowledge update request
    make_knowledge_update_request(unicycle_number_);

    RCLCPP_INFO(this->get_logger(), "Data collection completed. Moving to rendezvous position.");

    // Publish the desired pose for rendez-vous
    rendezvous_desired_pose_pub_->publish(*rendezvous_desired_place_);

    // Start rendezvous timeout timer
    rendezvous_timeout_timer_->reset();

    // Transition to the WAITING_FOR_RV state
    transition_to_state(FSM::WAITING_FOR_RV);
}

// Function to handle the WAITING_FOR_RV state
void TaskManager::handle_waiting_for_rv_state()
{
    if (!all_rendezvous_complete_[unicycle_number_])
    {
        // If rendez-vous is not complete for this unicycle, wait
        return;
    }

    // Check if all rendezvous are complete
    bool rendezvous_complete_ = true;

    // If there's a single unicycle that didn't complete rendez-vous
    for (int i = 0; i < num_unicycles_; ++i)
    {
        if (!all_rendezvous_complete_[i])
        {
            rendezvous_complete_ = false;
            break;
        }
    }

    if (rendezvous_complete_)
    {
        RCLCPP_INFO(this->get_logger(), "Rendez-vous complete for all unicycles. Exchanging data...");
        rendezvous_timeout_timer_->cancel();
        transition_to_state(FSM::EXCHANGING_DATA);
    }
    else if (rendezvous_failed_)
    {
        RCLCPP_WARN(this->get_logger(), "Rendez-vous failed for unicycle %d", unicycle_number_);
        transition_to_state(FSM::CHOOSING_NEXT_PLACE);
    }
}

// Function to handle the EXCHANGING_DATA state
void TaskManager::handle_exchanging_data_state()
{
    // Exchange data with other unicycles
    RCLCPP_INFO(this->get_logger(), "Exchanging data with other unicycles...");
    for (int i = 0; i < num_unicycles_; ++i)
    {
        if (i != unicycle_number_)
        {
            // Make knowledge update request
            make_knowledge_update_request(i);
        }
    }

    // Switch to the FINALIZING state
    transition_to_state(FSM::CHOOSING_NEXT_PLACE);
}

// Function to handle the CHOOSING_NEXT_PLACE state
void TaskManager::handle_choosing_next_place_state()
{
    // Choosing the next target place
    if (current_target_place_ < num_places_ - 1)
    {
        current_target_place_++;
    }
    else
    {
        current_target_place_ = 0;
    }

    RCLCPP_INFO(this->get_logger(), "Next target place is: %d", current_target_place_);

    // Transition to finalizing state
    transition_to_state(FSM::FINALIZING);
}

// Function to handle the FINALIZING state
void TaskManager::handle_finalizing_state()
{
    // Cleaning variables
    target_reached_ = false;
    rendezvous_complete_ = false;
    rendezvous_failed_ = false;

    // Reset the rendezvous complete flags
    std::fill(all_rendezvous_complete_.begin(), all_rendezvous_complete_.end(), false);

    // Publish the target pose based on the current target place
    target_pose_pub_->publish(*target_places_[current_target_place_]);

    RCLCPP_INFO(this->get_logger(), "Next target place for this unicycle is: %d", current_target_place_);

    // Transition to the GOING_TO_TARGET state
    transition_to_state(FSM::GOING_TO_TARGET);
}

// Callback to handle target reached
void TaskManager::target_reached_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    target_reached_ = msg->data;
}

void TaskManager::load_places_data()
{
    // Initialize target places vector
    target_places_.resize(num_places_);

    // Instantiate places objects according to num places
    for (int i = 0; i < num_places_; ++i)
    {
        target_places_[i] = std::make_shared<geometry_msgs::msg::Pose>();
    }

    rendezvous_desired_place_ = std::make_shared<geometry_msgs::msg::Pose>();

    // Load target places from parameters
    for (int i = 0; i < num_places_; ++i)
    {
        target_places_[i]->position.x = this->declare_parameter("target_place_" + std::to_string(i) + "_x", 0.0);
        target_places_[i]->position.z = this->declare_parameter("target_place_" + std::to_string(i) + "_z", 0.0);
    }

    // Load rendez-vous desired place
    rendezvous_desired_place_->position.x = this->declare_parameter("rv_desired_place_x", 0.0);
    rendezvous_desired_place_->position.z = this->declare_parameter("rv_desired_place_z", 0.0);

    // Print loaded places data
    for (int i = 0; i < num_places_; ++i)
    {
        RCLCPP_INFO(this->get_logger(), "Target place %d has been loaded: [%.2f, %.2f]",
                    i, target_places_[i]->position.z, target_places_[i]->position.x);
    }

    RCLCPP_INFO(this->get_logger(), "Rendez-vous desired place: [%.2f, %.2f]",
                rendezvous_desired_place_->position.z, rendezvous_desired_place_->position.x);
}

// Callback for rendezvous status according to the unicycle number
void TaskManager::rendezvous_status_callback(const std_msgs::msg::Bool::SharedPtr msg, const int unicycle_id)
{
    all_rendezvous_complete_[unicycle_id] = msg->data;
}

// Callback for unity simulation status
void TaskManager::unity_simulation_status_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    unity_simulation_ready_ = msg->data;
}

// Function to manage save knowledge requests
void TaskManager::make_knowledge_update_request(const int target_unicycle_id)
{
    if (target_unicycle_id < 0 || target_unicycle_id >= num_unicycles_)
    {
        RCLCPP_WARN(this->get_logger(), "Invalid target_unicycle_id: %d", target_unicycle_id);
        return;
    }

    auto client = knowledge_clients_[target_unicycle_id];
    if (!client)
    {
        RCLCPP_WARN(this->get_logger(), "Knowledge client for unicycle %d is not initialized", target_unicycle_id);
        return;
    }

    // Wait for service
    const auto wait_timeout = std::chrono::seconds(2);
    if (!client->wait_for_service(wait_timeout))
    {
        RCLCPP_WARN(this->get_logger(),
                    "Service for unicycle %d not available after %lds, skipping",
                    target_unicycle_id, wait_timeout.count());
        return;
    }

    // Prepare request
    auto request = std::make_shared<unicycle::srv::UpdateKnowledge::Request>();
    request->id = {static_cast<int8_t>(current_target_place_)};
    request->temperature = {static_cast<float>(sensors_data_.temperature)};
    request->humidity = {static_cast<float>(sensors_data_.humidity)};
    request->air_quality = {static_cast<int32_t>(sensors_data_.air_quality)};

    // Send asynchronously with callback
    client->async_send_request(
        request,
        [this, target_unicycle_id](rclcpp::Client<unicycle::srv::UpdateKnowledge>::SharedFuture future) {
            try
            {
                auto result = future.get();
                if (result && result->success)
                {
                    RCLCPP_INFO(this->get_logger(),
                                "Knowledge saved successfully on unicycle %d",
                                target_unicycle_id);
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(),
                                 "Knowledge save FAILED on unicycle %d (response null or success=false)",
                                 target_unicycle_id);
                }
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(this->get_logger(),
                             "Exception while processing response from unicycle %d: %s",
                             target_unicycle_id, e.what());
            }
        });

}

// Callback for temperature sensor
void TaskManager::temperature_sensor_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    sensors_data_.temperature = msg->data;
}

// Callback for humidity sensor
void TaskManager::humidity_sensor_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    sensors_data_.humidity = msg->data;
}

// Callback for air quality sensor
void TaskManager::air_quality_sensor_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
    sensors_data_.air_quality = msg->data;
}

// Callback to manage rendez-vous timeout
void TaskManager::rendezvous_timeout_callback()
{
    RCLCPP_WARN(this->get_logger(), "Rendez-vous timeout reached");
    rendezvous_failed_ = true;
    rendezvous_timeout_timer_->cancel();
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