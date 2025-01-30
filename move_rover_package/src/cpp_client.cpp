#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "move_rover_action/action/move_rover.hpp"

class RoverActionClient : public rclcpp::Node {
public:
    using MoveRover = move_rover_action::action::MoveRover;
    using GoalHandleMoveRover = rclcpp_action::ClientGoalHandle<MoveRover>;

    explicit RoverActionClient() : Node("rover_action_client") {
        this->action_client_ = rclcpp_action::create_client<MoveRover>(this, "move_rover");

        RCLCPP_INFO(this->get_logger(), "Waiting for action server...");
        if (!this->action_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available!");
            return;
        }

        send_goal(5.0);
    }

private:
    rclcpp_action::Client<MoveRover>::SharedPtr action_client_;

    void send_goal(double distance) {
        auto goal_msg = MoveRover::Goal();
        goal_msg.distance = distance;

        RCLCPP_INFO(this->get_logger(), "Sending goal...");

        auto send_goal_options = rclcpp_action::Client<MoveRover>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&RoverActionClient::goal_response_callback, this, std::placeholders::_1);
        send_goal_options.result_callback =
            std::bind(&RoverActionClient::get_result_callback, this, std::placeholders::_1);
        send_goal_options.feedback_callback =
            std::bind(&RoverActionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);

        this->action_client_->async_send_goal(goal_msg, send_goal_options);
    }

    void goal_response_callback(GoalHandleMoveRover::SharedPtr goal_handle) {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal rejected.");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted.");
        }
    }

    void get_result_callback(const GoalHandleMoveRover::WrappedResult &result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(this->get_logger(), "Goal reached successfully!");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal failed or was canceled.");
        }
        rclcpp::shutdown();
    }

    void feedback_callback(GoalHandleMoveRover::SharedPtr, const std::shared_ptr<const MoveRover::Feedback> feedback) {
        RCLCPP_INFO(this->get_logger(), "Feedback: %s", feedback->progress.c_str());
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RoverActionClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
