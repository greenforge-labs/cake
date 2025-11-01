#pragma once

#include <functional>
#include <memory>
#include <optional>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace cake {

template <typename ActionT> struct SingleGoalActionServerOptions {
    bool new_goals_replace_current_goal = false;
    std::function<bool(const typename ActionT::Goal &)> goal_validator =
        [](const typename ActionT::Goal & /*goal*/) -> bool { return true; };
};

template <typename ActionT> class SingleGoalActionServer {
  public:
    using GoalHandle = rclcpp_action::ServerGoalHandle<ActionT>;

    explicit SingleGoalActionServer(
        rclcpp::Node *node,
        const std::string &server_name,
        const std::optional<SingleGoalActionServerOptions<ActionT>> &options = std::nullopt
    )
        : node_(node), action_server_(nullptr), active_goal_handle_(nullptr), options_(options) {

        action_server_ = rclcpp_action::create_server<ActionT>(
            node_,
            server_name,
            [this](const auto uuid, auto goal) { return handle_goal(uuid, goal); },
            [this](auto goal_handle) { return handle_cancel(goal_handle); },
            [this](auto goal_handle) { return handle_accepted(goal_handle); }
        );

        RCLCPP_INFO(node_->get_logger(), "Action server '%s' initialized", server_name.c_str());
    }

    ~SingleGoalActionServer() {
        if (active_goal_handle_) {
            active_goal_handle_->abort(std::make_shared<typename ActionT::Result>());
            active_goal_handle_ = nullptr;
        }
    }

    void set_options(const SingleGoalActionServerOptions<ActionT> &options) { options_ = options; }

    const std::shared_ptr<const typename ActionT::Goal> get_active_goal() {
        if (!active_goal_handle_) {
            return nullptr;
        }

        return active_goal_handle_->get_goal();
    }

    void publish_feedback(std::shared_ptr<typename ActionT::Feedback> feedback) {
        if (!active_goal_handle_) {
            RCLCPP_WARN(node_->get_logger(), "Cannot publish feedback: no active goal");
            return;
        }
        active_goal_handle_->publish_feedback(feedback);
    }

    void succeed(std::shared_ptr<typename ActionT::Result> result) {
        if (!active_goal_handle_) {
            RCLCPP_WARN(node_->get_logger(), "Cannot succeed: no active goal");
            return;
        }
        active_goal_handle_->succeed(result);
        active_goal_handle_ = nullptr;
    }

    void abort(std::shared_ptr<typename ActionT::Result> result) {
        if (!active_goal_handle_) {
            RCLCPP_WARN(node_->get_logger(), "Cannot abort: no active goal");
            return;
        }
        active_goal_handle_->abort(result);
        active_goal_handle_ = nullptr;
    }

  private:
    rclcpp::Node *node_;
    rclcpp_action::Server<ActionT>::SharedPtr action_server_;
    std::shared_ptr<GoalHandle> active_goal_handle_;
    std::optional<SingleGoalActionServerOptions<ActionT>> options_;

    rclcpp_action::GoalResponse
    handle_goal(const rclcpp_action::GoalUUID & /*uuid*/, std::shared_ptr<const typename ActionT::Goal> goal) {
        if (!options_.has_value()) {
            RCLCPP_WARN(
                node_->get_logger(),
                "Rejecting goal: action server options not configured. Call set_options() to configure."
            );
            return rclcpp_action::GoalResponse::REJECT;
        }

        if (!options_.value().goal_validator(*goal)) {
            RCLCPP_WARN(node_->get_logger(), "Rejecting goal, goal is invalid");
            return rclcpp_action::GoalResponse::REJECT;
        }

        if (active_goal_handle_) {
            if (options_.value().new_goals_replace_current_goal) {
                RCLCPP_WARN(node_->get_logger(), "Cancelling current goal");
                handle_cancel(active_goal_handle_);
            } else {
                RCLCPP_WARN(node_->get_logger(), "Rejecting goal, another goal is active");
                return rclcpp_action::GoalResponse::REJECT;
            }
        }

        RCLCPP_INFO(node_->get_logger(), "Accepting goal");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(std::shared_ptr<GoalHandle> goal_handle) {
        RCLCPP_INFO(node_->get_logger(), "Received request to cancel goal");

        if (active_goal_handle_ && goal_handle->get_goal_id() == active_goal_handle_->get_goal_id()) {
            active_goal_handle_ = nullptr;
            RCLCPP_INFO(node_->get_logger(), "Goal canceled");
        }

        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(std::shared_ptr<GoalHandle> goal_handle) {
        active_goal_handle_ = goal_handle;
        RCLCPP_INFO(node_->get_logger(), "Goal accepted");
    }
};

template <typename ActionT>
std::shared_ptr<SingleGoalActionServer<ActionT>> create_single_goal_action_server(
    rclcpp::Node *node,
    const std::string &server_name,
    const std::optional<SingleGoalActionServerOptions<ActionT>> &options = std::nullopt
) {
    return std::make_shared<SingleGoalActionServer<ActionT>>(node, server_name, options);
}

template <typename ActionT, typename ContextType>
std::shared_ptr<SingleGoalActionServer<ActionT>> create_single_goal_action_server(
    std::shared_ptr<ContextType> context,
    const std::string &server_name,
    const std::optional<SingleGoalActionServerOptions<ActionT>> &options = std::nullopt
) {
    return create_single_goal_action_server(context->node.get(), server_name, options);
}

} // namespace cake
