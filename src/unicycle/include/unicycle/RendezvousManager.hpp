/*
    Rendez-vous Manager - Header File

    This file contains the declaration of the RendezvousManager class, which is responsible for managing rendez-vous point between unicycle in the system.

    Written by: Ivan Sollazzo
*/

#ifndef RENDEZVOUS_MANAGER_HPP
#define RENDEZVOUS_MANAGER_HPP

// Include necessary headers
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/bool.hpp>
#include <vector>

class RendezvousManager : public rclcpp::Node
{
    public:
        // Constructor
        explicit RendezvousManager(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    private:

        // Unicycle identifier
        std::string unicycle_id_;

        // Unicycle number
        int unicycle_number_;

        // Variable to store the number of unicycles
        int num_unicycles_;

        // Boolean array to track desired unicycles pose
        std::vector<bool> desired_poses_received_;

        // Array of poses to store current unicycles pose
        std::vector<geometry_msgs::msg::Pose::SharedPtr> current_poses_;

        // Subscriptions array according to number of unicycle
        std::vector<rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr> desired_poses_subs_;

        // Subscriptions array to track all unicycles pose
        std::vector<rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr> current_poses_subs_;

        // Publisher for rendez-vous point
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr target_pose_pub_;

        // Subscriber for target reached
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr target_reached_sub_;

        // Publisher for rendez-vous status
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr rendezvous_status_pub_;

        // Callback to track desired unicycles pose. We use a single callback and use the unicycle ID to differentiate between them.
        void desired_pose_received_callback(const geometry_msgs::msg::Pose::SharedPtr msg, const int unicycle_id);

        // Callback to check if target is reached
        void target_reached_callback(const std_msgs::msg::Bool::SharedPtr msg);

        // Callback to track current unicycles pose. We use a single callback and use the unicycle ID to differentiate between them.
        void current_pose_received_callback(const geometry_msgs::msg::Pose::SharedPtr msg, const int unicycle_id);

        // Defining the FSM (Finite State Machine) in order to control the task manager
        enum class FSM
        {
            IDLE,
            WAITING_FOR_DESIRED_PLACES,
            CALCULATING_RV_PLACE,
            GOING_TO_RV_PLACE,
            FINALIZING
        };

        // Variable to hold the current state of the FSM
        FSM current_state_ = FSM::IDLE;

        // Function to transition between states
        void transition_to_state(FSM new_state);

        // Defining some functions to handle the FSM states
        void handle_idle_state();
        void handle_waiting_for_desired_places_state();
        void handle_calculating_rv_place_state();
        void handle_going_to_rv_place_state();
        void handle_finalizing_state();

        // Variable to handle target reached
        bool target_reached_ = false;

        // Function to call the FSM
        void fsm();

        // Timer to periodically call the FSM
        rclcpp::TimerBase::SharedPtr fsm_timer_;

        // Array to hold the desired poses
        std::vector<geometry_msgs::msg::Pose::SharedPtr> desired_poses_;

        // Parameters to configure the rendezvous behavior
        double rendezvous_error_update_factor_;
        double rendezvous_error_threshold_;

        // Variables to store the rendez-vous errors
        double rendezvous_error_x_ = 0.0;
        double rendezvous_error_z_ = 0.0;

};

#endif // RENDEZVOUS_MANAGER_HPP