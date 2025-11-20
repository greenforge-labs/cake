#pragma once

#include <rclcpp/rclcpp.hpp>

#include "fixed_string.hpp"

namespace cake {

template <fixed_string node_name, auto extend_options = [](rclcpp::NodeOptions options) { return options; }>
class BaseNode {
  public:
    explicit BaseNode(const rclcpp::NodeOptions &options)
        : node_(std::make_shared<rclcpp::Node>(node_name.c_str(), extend_options(options))) {}

    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() const {
        return this->node_->get_node_base_interface();
    }

  protected:
    rclcpp::Node::SharedPtr node_;
};

} // namespace cake
