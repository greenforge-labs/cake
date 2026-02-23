#pragma once

#include <chrono>
#include <type_traits>

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>

#include "session.hpp"

namespace cake {

template <typename DurationRepT, typename DurationT, typename SessionType, typename CallbackT>
auto create_timer(
    std::shared_ptr<SessionType> sn,
    std::chrono::duration<DurationRepT, DurationT> period,
    CallbackT callback,
    rclcpp::CallbackGroup::SharedPtr group = nullptr
) {
    static_assert(std::is_base_of_v<Session, SessionType>, "SessionType must derive from cake::Session");

    bool autostart = (sn->node.get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

    std::weak_ptr<SessionType> weak_sn = sn;
    auto timer = rclcpp::create_timer(
        sn->node.shared_from_this(),
        sn->node.get_clock(),
        rclcpp::Duration(period),
        [weak_sn, callback]() {
            auto sn = weak_sn.lock();
            if (!sn)
                return;
            if (sn->node.get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
                callback(sn);
            }
        },
        group,
        autostart
    );

    sn->timers.push_back(timer);
    return timer;
}

template <typename DurationRepT, typename DurationT, typename SessionType, typename CallbackT>
auto create_wall_timer(
    std::shared_ptr<SessionType> sn,
    std::chrono::duration<DurationRepT, DurationT> period,
    CallbackT callback,
    rclcpp::CallbackGroup::SharedPtr group = nullptr,
    bool autostart = true
) {
    static_assert(std::is_base_of_v<Session, SessionType>, "SessionType must derive from cake::Session");

    bool effective_autostart =
        autostart && (sn->node.get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

    std::weak_ptr<SessionType> weak_sn = sn;
    auto timer = rclcpp::create_wall_timer(
        period,
        [weak_sn, callback]() {
            auto sn = weak_sn.lock();
            if (!sn)
                return;
            if (sn->node.get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
                callback(sn);
            }
        },
        group,
        sn->node.get_node_base_interface().get(),
        sn->node.get_node_timers_interface().get(),
        effective_autostart
    );

    sn->timers.push_back(timer);
    return timer;
}

} // namespace cake
