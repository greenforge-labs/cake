#pragma once

#include <memory>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

namespace cake {

struct Context {
    rclcpp::Node::SharedPtr node;
    std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions;
    std::vector<rclcpp::TimerBase::SharedPtr> timers;
    std::vector<std::thread> threads;

    explicit Context(rclcpp::Node::SharedPtr node) : node(node) {}
};

template <typename ContextType> std::shared_ptr<ContextType> init_ctx(rclcpp::Node::SharedPtr node) {
    static_assert(std::is_base_of_v<Context, ContextType>, "ContextType must be a child of Context");
    return std::make_shared<ContextType>(node);
}

} // namespace cake
