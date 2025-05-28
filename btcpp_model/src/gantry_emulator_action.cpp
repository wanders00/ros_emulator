#include "btcpp_model/gantry_emulator_action.hpp"
#include <sstream>

GantryEmulatorAction::GantryEmulatorAction(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node)
    : BT::SyncActionNode(name, config), node_(node) {
    RCLCPP_INFO(node_->get_logger(), "GantryEmulatorAction constructed: %s", name.c_str());
    if (!rclcpp::ok())
        rclcpp::init(0, nullptr);
    client_ = node_->create_client<emulation_msgs::srv::TriggerGantry>("/gantry_emulator_service");
}

BT::NodeStatus GantryEmulatorAction::tick() {
    RCLCPP_INFO(node_->get_logger(), "GantryEmulatorAction::tick() called");
    auto request = std::make_shared<emulation_msgs::srv::TriggerGantry::Request>();

    getInput("command", request->command);
    getInput("position", request->position);
    RCLCPP_INFO(node_->get_logger(), "Command: %s, Position: %s", request->command.c_str(), request->position.c_str());

    // Set emulation parameters from environment variables or default to 0
    auto get_env_or_zero = [](const char* var) -> int {
        const char* val = std::getenv(var);
        if (val) {
            try {
                return std::stoi(val);
            } catch (...) {
                return 0;
            }
        }
        return 0;
    };
    request->emulated_response.emulate_execution_time = get_env_or_zero("GANTRY_EMULATE_EXECUTION_TIME");
    request->emulated_response.emulated_execution_time = get_env_or_zero("GANTRY_EMULATED_EXECUTION_TIME");
    request->emulated_response.emulate_failure_rate = get_env_or_zero("GANTRY_EMULATE_FAILURE_RATE");
    request->emulated_response.emulated_failure_rate = get_env_or_zero("GANTRY_EMULATED_FAILURE_RATE");
    request->emulated_response.emulate_failure_cause = get_env_or_zero("GANTRY_EMULATE_FAILURE_CAUSE");

    // Set emulated_failure_cause from env if present
    const char* causes_env = std::getenv("GANTRY_EMULATED_FAILURE_CAUSE");
    if (causes_env) {
        std::stringstream ss(causes_env);
        std::string cause;
        request->emulated_response.emulated_failure_cause.clear();
        while (std::getline(ss, cause, ',')) {
            request->emulated_response.emulated_failure_cause.push_back(cause);
        }
    }

    if (!client_->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_ERROR(node_->get_logger(), "GantryEmulatorAction: Service not available");
        return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(node_->get_logger(), "Sending request to gantry emulator service");
    auto future = client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, future) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node_->get_logger(), "GantryEmulatorAction: Service call failed");
        return BT::NodeStatus::FAILURE;
    }

    auto response = future.get();
    RCLCPP_INFO(node_->get_logger(), "GantryEmulatorAction: Service response: %s", response->success ? "SUCCESS" : "FAILURE");
    return response->success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::PortsList GantryEmulatorAction::providedPorts() {
    return {
        BT::InputPort<std::string>("command"),
        BT::InputPort<std::string>("position")
    };
}