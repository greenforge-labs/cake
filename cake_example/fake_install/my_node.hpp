#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include <cake/base_node.hpp>
#include <cake/context.hpp>
#include <cake/subscriber.hpp>

namespace cake_example::my_node {

template <typename ContextType> struct MyNodePublishers {
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr some_topic;
};

template <typename ContextType> struct MyNodeSubscribers {
    std::shared_ptr<cake::Subscriber<std_msgs::msg::Bool, ContextType>> other_topic;
};

template <typename DerivedContextType> struct MyNodeContext : cake::Context {
    MyNodePublishers<DerivedContextType> publishers;
    MyNodeSubscribers<DerivedContextType> subscribers;
};

template <
    typename ContextType,
    auto init_func,
    auto extend_options = [](rclcpp::NodeOptions options) { return options; }>
class MyNodeBase : public cake::BaseNode<"my_node", extend_options> {
  public:
    explicit MyNodeBase(const rclcpp::NodeOptions &options) : cake::BaseNode<"my_node", extend_options>(options) {
        static_assert(
            std::is_base_of_v<MyNodeContext<ContextType>, ContextType>, "ContextType must be a child of MyNodeContext"
        );

        // init context
        auto ctx = std::make_shared<ContextType>();
        ctx->node = this->node_;

        // init publishers
        ctx->publishers.some_topic = ctx->node->template create_publisher<std_msgs::msg::String>("some_topic", 10);

        // init subscribers
        ctx->subscribers.other_topic = cake::create_subscriber<std_msgs::msg::Bool>(ctx, "other_topic", 10);

        // TODO init services and actions

        init_func(ctx);
    }
};

} // namespace cake_example::my_node
