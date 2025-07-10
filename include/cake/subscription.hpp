#pragma once

#include <rclcpp/rclcpp.hpp>

namespace cake {

template <typename MessageT, typename CallbackType>
auto create_subscription(
    rclcpp::Node::SharedPtr node,
    const std::string& topic_name,
    const rclcpp::QoS& qos,
    CallbackType callback) {
    return node->create_subscription<MessageT>(
        topic_name, qos, [node, callback](typename MessageT::SharedPtr msg) {
            callback(node, msg);
        }
    );
}

} // namespace cake