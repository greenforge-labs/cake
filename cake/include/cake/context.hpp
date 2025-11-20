#pragma once

#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

namespace cake {

struct Context {
    rclcpp::Node::SharedPtr node;
    std::vector<rclcpp::TimerBase::SharedPtr> timers;
    std::vector<std::thread> threads;
};

} // namespace cake
