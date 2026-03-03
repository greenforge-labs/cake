#pragma once

#include <chrono>
#include <type_traits>

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>

#include "session.hpp"

namespace cake {

/// Create a timer that fires a session-aware callback at the given period.
///
/// The timer is created in a **cancelled** state and will be started automatically
/// by the lifecycle activate transition. To start it manually, call `timer->reset()`.
///
/// The callback only fires while the node is in the ACTIVE lifecycle state.
/// The timer is automatically registered with the session for lifecycle management
/// (cancelled on deactivate, reset on activate).
template <typename DurationRepT, typename DurationT, typename SessionType, typename CallbackT>
auto create_timer(
    std::shared_ptr<SessionType> sn,
    std::chrono::duration<DurationRepT, DurationT> period,
    CallbackT callback,
    rclcpp::CallbackGroup::SharedPtr group = nullptr
) {
    static_assert(std::is_base_of_v<Session, SessionType>, "SessionType must derive from cake::Session");

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
        /* autostart = */ false
    );

    sn->timers.push_back(timer);
    return timer;
}

/// Create a wall timer that fires a session-aware callback at the given period.
///
/// The timer is created in a **cancelled** state and will be started automatically
/// by the lifecycle activate transition. To start it manually, call `timer->reset()`.
///
/// The callback only fires while the node is in the ACTIVE lifecycle state.
/// The timer is automatically registered with the session for lifecycle management
/// (cancelled on deactivate, reset on activate).
template <typename DurationRepT, typename DurationT, typename SessionType, typename CallbackT>
auto create_wall_timer(
    std::shared_ptr<SessionType> sn,
    std::chrono::duration<DurationRepT, DurationT> period,
    CallbackT callback,
    rclcpp::CallbackGroup::SharedPtr group = nullptr
) {
    static_assert(std::is_base_of_v<Session, SessionType>, "SessionType must derive from cake::Session");

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
        /* autostart = */ false
    );

    sn->timers.push_back(timer);
    return timer;
}

} // namespace cake
