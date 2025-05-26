#include "btcpp_model/robot_emulator_action.hpp"

#include <sstream>

RobotEmulatorAction::RobotEmulatorAction(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node)
    : BT::SyncActionNode(name, config), node_(node) {
    RCLCPP_INFO(node_->get_logger(), "RobotEmulatorAction constructed: %s", name.c_str());
    if (!rclcpp::ok())
        rclcpp::init(0, nullptr);
    client_ = node_->create_client<emulation_msgs::srv::TriggerRobot>("/robot_emulator_service");
}

BT::NodeStatus RobotEmulatorAction::tick() {
    RCLCPP_INFO(node_->get_logger(), "RobotEmulatorAction::tick() called");
    auto request = std::make_shared<emulation_msgs::srv::TriggerRobot::Request>();

    getInput("command", request->command);
    getInput("position", request->position);
    RCLCPP_INFO(node_->get_logger(), "Command: %s, Position: %s", request->command.c_str(), request->position.c_str());

    request->emulated_response.emulate_execution_time = getInput<uint8_t>("emulate_execution_time").value_or(0);
    request->emulated_response.emulated_execution_time = getInput<int32_t>("emulated_execution_time").value_or(0);
    request->emulated_response.emulate_failure_rate = getInput<uint8_t>("emulate_failure_rate").value_or(0);
    request->emulated_response.emulated_failure_rate = getInput<int32_t>("emulated_failure_rate").value_or(0);
    request->emulated_response.emulate_failure_cause = getInput<uint8_t>("emulate_failure_cause").value_or(0);

    if (request->emulated_response.emulate_failure_cause != 0) {
        std::string causes;
        if (getInput("emulated_failure_cause", causes)) {
            std::stringstream ss(causes);
            std::string cause;
            while (std::getline(ss, cause, ','))
                request->emulated_response.emulated_failure_cause.push_back(cause);
            RCLCPP_INFO(node_->get_logger(), "Emulated failure causes: %s", causes.c_str());
        }
    }

    if (!client_->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_ERROR(node_->get_logger(), "RobotEmulatorAction: Service not available");
        return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(node_->get_logger(), "Sending request to robot emulator service");
    auto future = client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, future) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node_->get_logger(), "RobotEmulatorAction: Service call failed");
        return BT::NodeStatus::FAILURE;
    }

    auto response = future.get();
    RCLCPP_INFO(node_->get_logger(), "RobotEmulatorAction: Service response: %s", response->success ? "SUCCESS" : "FAILURE");
    return response->success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::PortsList RobotEmulatorAction::providedPorts() {
    return {
        BT::InputPort<std::string>("command"),
        BT::InputPort<std::string>("position"),
        BT::InputPort<uint8_t>("emulate_execution_time"),
        BT::InputPort<int32_t>("emulated_execution_time"),
        BT::InputPort<uint8_t>("emulate_failure_rate"),
        BT::InputPort<int32_t>("emulated_failure_rate"),
        BT::InputPort<uint8_t>("emulate_failure_cause"),
        BT::InputPort<std::string>("emulated_failure_cause")};
}