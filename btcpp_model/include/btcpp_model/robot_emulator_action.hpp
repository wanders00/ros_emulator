#pragma once

#include <behaviortree_cpp/action_node.h>

#include <emulation_msgs/srv/trigger_robot.hpp>
#include <rclcpp/rclcpp.hpp>

class RobotEmulatorAction : public BT::SyncActionNode {
   public:
    RobotEmulatorAction(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node);
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts();

   private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<emulation_msgs::srv::TriggerRobot>::SharedPtr client_;
};