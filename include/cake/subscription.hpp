#pragma once

#include <rclcpp/rclcpp.hpp>

#include "context.hpp"

namespace cake {

template <typename MessageT, typename ContextType, typename CallbackType>
auto create_subscription(
    std::shared_ptr<ContextType> context, const std::string &topic_name, const rclcpp::QoS &qos, CallbackType callback
) {
    static_assert(std::is_base_of_v<Context, ContextType>, "ContextType must be a child of Context");
    auto subscription = context->node->template create_subscription<MessageT>(
        topic_name, qos, [context, callback](typename MessageT::SharedPtr msg) { callback(context, msg); }
    );
    context->subscriptions.push_back(subscription);
    return subscription;
}

} // namespace cake
