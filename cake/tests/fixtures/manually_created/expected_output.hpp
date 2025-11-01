// auto-generated DO NOT EDIT

#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include <cake/base_node.hpp>
#include <cake/context.hpp>
#include <cake/subscriber.hpp>

namespace test_package::manual_node {

template <typename ContextType> struct ManualNodePublishers {
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr auto_topic;
};

template <typename ContextType> struct ManualNodeSubscribers {
    std::shared_ptr<cake::Subscriber<std_msgs::msg::Bool, ContextType>> auto_sub;
};

template <typename DerivedContextType> struct ManualNodeContext : cake::Context {
    ManualNodePublishers<DerivedContextType> publishers;
    ManualNodeSubscribers<DerivedContextType> subscribers;
};


template <
    typename ContextType,
    auto init_func,
    auto extend_options = [](rclcpp::NodeOptions options) { return options; }>
class ManualNodeBase : public cake::BaseNode<"manual_node", extend_options> {
  public:
    explicit ManualNodeBase(const rclcpp::NodeOptions &options) : cake::BaseNode<"manual_node", extend_options>(options) {
        static_assert(
            std::is_base_of_v<ManualNodeContext<ContextType>, ContextType>, "ContextType must be a child of ManualNodeContext"
        );

        // init context
        auto ctx = std::make_shared<ContextType>();
        ctx->node = this->node_;

        // init publishers
        ctx->publishers.auto_topic = ctx->node->template create_publisher<std_msgs::msg::String>("auto_topic", 10);
        // init subscribers
        ctx->subscribers.auto_sub = cake::create_subscriber<std_msgs::msg::Bool>(ctx, "auto_sub", 10);
        // TODO init services and actions

        init_func(ctx);
    }
};

} // namespace test_package::manual_node
