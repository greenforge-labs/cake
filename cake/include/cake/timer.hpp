#pragma once

#include <chrono>
#include <type_traits>

#include <lifecycle_msgs/msg/state.hpp>
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
    static_assert(std::is_base_of_v<Context, ContextType>, "ContextType must derive from cake::Context");

    bool autostart = (context->node->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

    std::weak_ptr<ContextType> weak_ctx = context;
    auto timer = rclcpp::create_timer(
        context->node,
        context->node->get_clock(),
        rclcpp::Duration(period),
        [weak_ctx, callback]() {
            auto ctx = weak_ctx.lock();
            if (!ctx)
                return;
            if (ctx->node->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
                callback(ctx);
            }
        },
        group,
        autostart
    );

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
    static_assert(std::is_base_of_v<Context, ContextType>, "ContextType must derive from cake::Context");

    bool effective_autostart =
        autostart && (context->node->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

    std::weak_ptr<ContextType> weak_ctx = context;
    auto timer = rclcpp::create_wall_timer(
        period,
        [weak_ctx, callback]() {
            auto ctx = weak_ctx.lock();
            if (!ctx)
                return;
            if (ctx->node->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
                callback(ctx);
            }
        },
        group,
        context->node->get_node_base_interface().get(),
        context->node->get_node_timers_interface().get(),
        effective_autostart
    );

    context->timers.push_back(timer);
    return timer;
}

} // namespace cake
