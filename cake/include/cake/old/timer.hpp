#pragma once

#pragma message(                                                                                                               \
    "WARNING: you are using the old deprecated version of cake. Please use the interface.yaml code generation system instead." \
)

#include <chrono>
#include <rclcpp/rclcpp.hpp>

#include "context.hpp"

namespace cake {

template <typename DurationRepT, typename DurationT, typename ContextType, typename CallbackT>
auto create_timer(
    std::shared_ptr<ContextType> context,
    std::chrono::duration<DurationRepT, DurationT> period,
    CallbackT callback,
    rclcpp::CallbackGroup::SharedPtr group = nullptr
) {
    static_assert(std::is_base_of_v<Context, ContextType>, "ContextType must be a child of Context");
    auto timer = context->node->template create_timer(period, [context, callback]() { callback(context); }, group);
    context->timers.push_back(timer);
    return timer;
}

template <typename DurationRepT, typename DurationT, typename ContextType, typename CallbackT>
auto create_wall_timer(
    std::shared_ptr<ContextType> context,
    std::chrono::duration<DurationRepT, DurationT> period,
    CallbackT callback,
    rclcpp::CallbackGroup::SharedPtr group = nullptr,
    bool autostart = true
) {
    static_assert(std::is_base_of_v<Context, ContextType>, "ContextType must be a child of Context");
    auto timer = context->node->template create_wall_timer(
        period, [context, callback]() { callback(context); }, group, autostart
    );
    context->timers.push_back(timer);
    return timer;
}

} // namespace cake
