#include <chrono>
#include <cstdlib>
#include <emulation_msgs/srv/trigger_robot.hpp>
#include <memory>
#include <sstream>
#include <string>

#include "plansys2_executor/ActionExecutorClient.hpp"
#include "plansys2_model/util.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

class RobotMoveActionNode : public plansys2::ActionExecutorClient {
   public:
    RobotMoveActionNode()
        : plansys2::ActionExecutorClient("move_robot", 1s) {
        client_ = this->create_client<emulation_msgs::srv::TriggerRobot>("/robot_emulator_service");
        RCLCPP_INFO(this->get_logger(), "RobotMoveActionNode constructed");
    }

   private:
    rclcpp::Client<emulation_msgs::srv::TriggerRobot>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr polling_timer_;
    rclcpp::Time request_start_time_;
    rclcpp::Client<emulation_msgs::srv::TriggerRobot>::SharedFuture future_;
    std::atomic<bool> waiting_for_response_{false};

    void do_work() override {
        if (waiting_for_response_) {
            return;
        }

        RCLCPP_INFO(this->get_logger(), "RobotMoveActionNode::do_work() called");

        const auto& args = get_arguments();
        std::string position = args.size() > 2 ? args[2] : "";

        auto request = std::make_shared<emulation_msgs::srv::TriggerRobot::Request>();
        request->command = "move";
        request->position = position;
        fill_emulated_response_from_env("ROBOT", request->emulated_response);

        if (!client_->wait_for_service(std::chrono::seconds(2))) {
            RCLCPP_ERROR(this->get_logger(), "Service not available");
            finish(false, 1.0, "Service not available");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Sending request to robot emulator service");

        future_ = client_->async_send_request(request).future.share();
        waiting_for_response_ = true;
        request_start_time_ = this->now();

        polling_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            [this]() {
                if (!waiting_for_response_) return;

                if ((this->now() - request_start_time_).seconds() > 10.0) {
                    waiting_for_response_ = false;
                    polling_timer_->cancel();
                    finish(false, 1.0, "Timeout waiting for robot response");
                    return;
                }

                if (future_.valid() &&
                    future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
                    waiting_for_response_ = false;
                    polling_timer_->cancel();

                    auto response = future_.get();
                    if (response->success) {
                        finish(true, 1.0, response->info);
                    } else {
                        finish(false, 1.0, response->failure_cause + ": " + response->info);
                    }
                }
            });
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto action = std::make_shared<RobotMoveActionNode>();
    action->set_parameter(rclcpp::Parameter("action_name", "move_robot"));
    action->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    rclcpp::spin(action->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
