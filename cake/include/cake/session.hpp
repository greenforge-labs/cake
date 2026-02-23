#pragma once

#include <vector>

#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace cake {

struct Session {
    rclcpp_lifecycle::LifecycleNode &node;
    std::vector<rclcpp::TimerBase::SharedPtr> timers;
    explicit Session(rclcpp_lifecycle::LifecycleNode &node) : node(node) {}
};

} // namespace cake
