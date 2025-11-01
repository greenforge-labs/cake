#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>

#include <cake/base_node.hpp>
#include <cake/context.hpp>

template <typename ContextType> struct PubNodePublishers {
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr counter;
};

template <typename ContextType> struct PubNodeSubscribers {};

template <typename DerivedContextType> struct PubNodeContext : cake::Context {
    PubNodePublishers<DerivedContextType> publishers;
    PubNodeSubscribers<DerivedContextType> subscribers;
};

template <
    typename ContextType,
    auto init_func,
    auto extend_options = [](rclcpp::NodeOptions options) { return options; }>
class PubNodeBase : public cake::BaseNode<"pub_node", extend_options> {
  public:
    explicit PubNodeBase(const rclcpp::NodeOptions &options) : cake::BaseNode<"pub_node", extend_options>(options) {
        static_assert(
            std::is_base_of_v<PubNodeContext<ContextType>, ContextType>, "ContextType must be a child of PubNodeContext"
        );

        // init context
        auto ctx = std::make_shared<ContextType>();
        ctx->node = this->node_;

        // init publishers
        ctx->publishers.status = ctx->node->template create_publisher<std_msgs::msg::String>("status", 10);
        ctx->publishers.counter = ctx->node->template create_publisher<std_msgs::msg::Int32>("counter", 5);
        // TODO init services and actions

        init_func(ctx);
    }
};
