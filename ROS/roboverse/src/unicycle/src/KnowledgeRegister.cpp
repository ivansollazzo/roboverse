/*
    Knowledge Register - Source File

    This file contains the implementation of the KnowledgeRegister class, which is responsible for managing knowledge on the unicycle.

    Written by: Ivan Sollazzo
*/


// Include the header only
#include <unicycle/KnowledgeRegister.hpp>

KnowledgeRegister::KnowledgeRegister() : Node("knowledge_register") {

  // Get the unicycle ID
  unicycle_id_ = this->declare_parameter<std::string>("unicycle_id", "unicycle_0");

  // Create the service for saving knowledge
  service_ = this->create_service<unicycle::srv::UpdateKnowledge>(unicycle_id_ +
    "/knowledge/update_knowledge",
    std::bind(&KnowledgeRegister::save_callback, this, std::placeholders::_1, std::placeholders::_2));
}

// Callback function for the service
void KnowledgeRegister::save_callback(
  const std::shared_ptr<unicycle::srv::UpdateKnowledge::Request> request,
  std::shared_ptr<unicycle::srv::UpdateKnowledge::Response> response)
{
  try {

    // If request is invalid, return
    if (request->id.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Request contains no IDs.");
      response->success = false;
      return;
    }

    size_t n = request->temperature.size();
    if (n == 0 || request->humidity.size() != n || request->air_quality.size() != n || request->id.size() != n) {
      RCLCPP_ERROR(this->get_logger(),
        "Mismatched vector sizes: id=%zu temp=%zu hum=%zu air=%zu",
        request->id.size(), request->temperature.size(),
        request->humidity.size(), request->air_quality.size());
      response->success = false;
      return;
    }

    // Get the base path
    std::string package_path = ament_index_cpp::get_package_prefix("unicycle");
    if (package_path.empty()) {
      response->success = false;
      return;
    }
    
    std::string dir_path = package_path + "/data/" + unicycle_id_;
    std::string file_path = dir_path + "/knowledge_" + std::to_string(request->id[0]) + ".csv";

    // Create directory
    fs::create_directories(dir_path);

    // Write to file
    bool file_exists = fs::exists(file_path);
    std::ofstream file(file_path, file_exists ? std::ios::app : std::ios::out);

    if (!file_exists) {
      RCLCPP_INFO(this->get_logger(), "Creating new file: %s", file_path.c_str());
      file << "id,temperature,humidity,air_quality\n";
    } else {
      RCLCPP_INFO(this->get_logger(), "Appending to existing file: %s", file_path.c_str());
    }

    for (size_t i = 0; i < n; ++i) {
      file << static_cast<int>(request->id[i]) << ","
           << std::fixed << std::setprecision(2)
           << static_cast<float>(request->temperature[i]) << ","
           << static_cast<float>(request->humidity[i]) << ","
           << static_cast<int>(request->air_quality[i]) << "\n";
    }

    file.close();
    RCLCPP_INFO(this->get_logger(), "Knowledge saved to: %s", file_path.c_str());
    response->success = true;

  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Error saving knowledge: %s", e.what());
    response->success = false;
  }
}

// Main function
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KnowledgeRegister>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}