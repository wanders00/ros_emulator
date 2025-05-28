#pragma once
#include <plansys2_executor/ActionExecutorClient.hpp>
#include <rclcpp/rclcpp.hpp>

class GantryEmulatorAction : public plansys2::ActionExecutorClient {
public:
  GantryEmulatorAction(const std::string & action_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
protected:
  void do_work() override;
};
