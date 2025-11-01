// auto-generated DO NOT EDIT

#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <cake/base_node.hpp>
#include <cake/context.hpp>
#include <cake/subscriber.hpp>

namespace test_package::simple_node {

template <typename ContextType> struct SimpleNodePublishers {
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel;
};

template <typename ContextType> struct SimpleNodeSubscribers {
    std::shared_ptr<cake::Subscriber<nav_msgs::msg::Odometry, ContextType>> odom;
};

template <typename DerivedContextType> struct SimpleNodeContext : cake::Context {
    SimpleNodePublishers<DerivedContextType> publishers;
    SimpleNodeSubscribers<DerivedContextType> subscribers;
};


template <
    typename ContextType,
    auto init_func,
    auto extend_options = [](rclcpp::NodeOptions options) { return options; }>
class SimpleNodeBase : public cake::BaseNode<"simple_node", extend_options> {
  public:
    explicit SimpleNodeBase(const rclcpp::NodeOptions &options) : cake::BaseNode<"simple_node", extend_options>(options) {
        static_assert(
            std::is_base_of_v<SimpleNodeContext<ContextType>, ContextType>, "ContextType must be a child of SimpleNodeContext"
        );

        // init context
        auto ctx = std::make_shared<ContextType>();
        ctx->node = this->node_;

        // init publishers
        ctx->publishers.cmd_vel = ctx->node->template create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        // init subscribers
        ctx->subscribers.odom = cake::create_subscriber<nav_msgs::msg::Odometry>(ctx, "/odom", 10);
        // TODO init services and actions

        init_func(ctx);
    }
};

} // namespace test_package::simple_node
