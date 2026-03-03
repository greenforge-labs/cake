#pragma once
#include <chrono>
#include <future>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace cake {

template <typename ServiceT>
typename ServiceT::Response::SharedPtr call_sync(
    typename rclcpp::Client<ServiceT>::SharedPtr client,
    typename ServiceT::Request::SharedPtr request,
    std::chrono::nanoseconds timeout = std::chrono::seconds(5)
) {
    auto future = client->async_send_request(request);
    if (future.wait_for(timeout) != std::future_status::ready) {
        return nullptr;
    }
    return future.get();
}

template <typename ActionT>
typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr send_goal_sync(
    typename rclcpp_action::Client<ActionT>::SharedPtr client,
    const typename ActionT::Goal &goal,
    const typename rclcpp_action::Client<ActionT>::SendGoalOptions &options = {},
    std::chrono::nanoseconds timeout = std::chrono::seconds(5)
) {
    auto future = client->async_send_goal(goal, options);
    if (future.wait_for(timeout) != std::future_status::ready) {
        return nullptr;
    }
    return future.get();
}

template <typename ActionT>
std::optional<typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult> get_result_sync(
    typename rclcpp_action::Client<ActionT>::SharedPtr client,
    typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr goal_handle,
    std::chrono::nanoseconds timeout = std::chrono::minutes(5)
) {
    auto future = client->async_get_result(goal_handle);
    if (future.wait_for(timeout) != std::future_status::ready) {
        return std::nullopt;
    }
    return future.get();
}

template <typename ActionT>
typename rclcpp_action::Client<ActionT>::CancelResponse::SharedPtr cancel_goal_sync(
    typename rclcpp_action::Client<ActionT>::SharedPtr client,
    typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr goal_handle,
    std::chrono::nanoseconds timeout = std::chrono::seconds(5)
) {
    auto future = client->async_cancel_goal(goal_handle);
    if (future.wait_for(timeout) != std::future_status::ready) {
        return nullptr;
    }
    return future.get();
}

} // namespace cake
