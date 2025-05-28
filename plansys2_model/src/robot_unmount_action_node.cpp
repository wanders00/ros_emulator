#include "plansys2_model/robot_emulator_action.hpp"
#include <rclcpp/rclcpp.hpp>
#include <emulation_msgs/srv/trigger_robot.hpp>
#include <cstdlib>
#include <sstream>
#include <chrono>
#include <memory>
#include <string>

using namespace std::chrono_literals;

class RobotUnmountActionNode : public plansys2::ActionExecutorClient {
public:
  RobotUnmountActionNode()
    : plansys2::ActionExecutorClient("unmount", 100ms) {
    client_ = this->create_client<emulation_msgs::srv::TriggerRobot>("/robot_emulator_service");
    RCLCPP_INFO(this->get_logger(), "RobotUnmountActionNode constructed");
    RCLCPP_INFO(this->get_logger(), "RobotEmulatorAction registered for 'unmount'");
  }
protected:
  void do_work() override {
    RCLCPP_INFO(this->get_logger(), "RobotUnmountActionNode::do_work() called");
    const auto & args = get_arguments();
    std::string command = get_action_name();
    std::string position;
    if (!args.empty()) {
      position = args[0];
    }
    RCLCPP_INFO(this->get_logger(), "Command: %s, Position: %s", command.c_str(), position.c_str());
    auto request = std::make_shared<emulation_msgs::srv::TriggerRobot::Request>();
    request->command = command;
    request->position = position;
    auto get_env_or_zero = [](const char* var) -> int {
      const char* val = std::getenv(var);
      if (val) {
        try { return std::stoi(val); } catch (...) { return 0; }
      }
      return 0;
    };
    request->emulated_response.emulate_execution_time = get_env_or_zero("ROBOT_EMULATE_EXECUTION_TIME");
    request->emulated_response.emulated_execution_time = get_env_or_zero("ROBOT_EMULATED_EXECUTION_TIME");
    request->emulated_response.emulate_failure_rate = get_env_or_zero("ROBOT_EMULATE_FAILURE_RATE");
    request->emulated_response.emulated_failure_rate = get_env_or_zero("ROBOT_EMULATED_FAILURE_RATE");
    request->emulated_response.emulate_failure_cause = get_env_or_zero("ROBOT_EMULATE_FAILURE_CAUSE");
    const char* causes_env = std::getenv("ROBOT_EMULATED_FAILURE_CAUSE");
    if (causes_env) {
      std::stringstream ss(causes_env);
      std::string cause;
      request->emulated_response.emulated_failure_cause.clear();
      while (std::getline(ss, cause, ',')) {
        request->emulated_response.emulated_failure_cause.push_back(cause);
      }
    }
    if (!client_->wait_for_service(std::chrono::seconds(2))) {
      RCLCPP_ERROR(this->get_logger(), "Service not available");
      finish(false, 1.0, "Service not available");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Sending request to robot emulator service");
    auto future = client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Service call failed");
      finish(false, 1.0, "Service call failed");
      return;
    }
    auto response = future.get();
    RCLCPP_INFO(this->get_logger(), "Service response: %s", response->success ? "SUCCESS" : "FAILURE");
    if (response->success) {
      finish(true, 1.0, response->info);
    } else {
      finish(false, 1.0, response->failure_cause + ": " + response->info);
    }
  }
private:
  rclcpp::Client<emulation_msgs::srv::TriggerRobot>::SharedPtr client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto action = std::make_shared<RobotUnmountActionNode>();
  action->set_parameter(rclcpp::Parameter("action_name", "unmount"));
  action->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  rclcpp::spin(action->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
