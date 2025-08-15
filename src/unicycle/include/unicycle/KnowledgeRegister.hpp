/*
    Knowledge Register - Header File

    This file contains the declaration of the KnowledgeRegister class, which is responsible for managing the knowledge on the unicycle.

    Written by: Ivan Sollazzo
*/

#ifndef KNOWLEDGE_REGISTER_HPP
#define KNOWLEDGE_REGISTER_HPP

// Include necessary headers
#include "rclcpp/rclcpp.hpp"
#include "unicycle/srv/update_knowledge.hpp"
#include <ament_index_cpp/get_package_prefix.hpp>

// Standard library headers
#include <filesystem>
#include <fstream>
#include <iomanip>

// Namespace alias for filesystem
namespace fs = std::filesystem;

class KnowledgeRegister : public rclcpp::Node {
public:
  KnowledgeRegister();

private:

  // Unique identifier for the robot
  std::string unicycle_id_;

  // Callback function for the service
  void save_callback(
    const std::shared_ptr<unicycle::srv::UpdateKnowledge::Request> request,
    std::shared_ptr<unicycle::srv::UpdateKnowledge::Response> response);

  // Service to save knowledge
  rclcpp::Service<unicycle::srv::UpdateKnowledge>::SharedPtr service_;
};

#endif  // KNOWLEDGE_REGISTER_HPP