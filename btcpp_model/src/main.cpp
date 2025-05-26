#include <behaviortree_cpp/bt_factory.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include "btcpp_model/gantry_emulator_action.hpp"
#include "btcpp_model/robot_emulator_action.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("btcpp_model"), "Node initialization started.");

    auto node = rclcpp::Node::make_shared("btcpp_model");
    RCLCPP_INFO(node->get_logger(), "Node created.");

    BT::BehaviorTreeFactory factory;
    factory.registerBuilder<RobotEmulatorAction>(
        "RobotEmulatorAction",
        [node](const std::string& name, const BT::NodeConfig& config) {
            RCLCPP_INFO(node->get_logger(), "Registering RobotEmulatorAction node: %s", name.c_str());
            return std::make_unique<RobotEmulatorAction>(name, config, node);
        });
    factory.registerBuilder<GantryEmulatorAction>(
        "GantryEmulatorAction",
        [node](const std::string& name, const BT::NodeConfig& config) {
            RCLCPP_INFO(node->get_logger(), "Registering GantryEmulatorAction node: %s", name.c_str());
            return std::make_unique<GantryEmulatorAction>(name, config, node);
        });

    std::string tree_file = ament_index_cpp::get_package_share_directory("btcpp_model") +
                            "/behavior_trees/example_tree.xml";
    RCLCPP_INFO(node->get_logger(), "Loading behavior tree from: %s", tree_file.c_str());
    auto tree = factory.createTreeFromFile(tree_file);

    /*
    while (rclcpp::ok() && tree.rootNode()->status() == BT::NodeStatus::IDLE) {
        tree.tickOnce();
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    */

    RCLCPP_INFO(node->get_logger(), "Ticking the behavior tree once.");
    tree.tickOnce();
    rclcpp::spin_some(node);

    RCLCPP_INFO(node->get_logger(), "Shutting down node.");
    rclcpp::shutdown();
    return 0;
}