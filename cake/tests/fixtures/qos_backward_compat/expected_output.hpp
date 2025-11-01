// auto-generated DO NOT EDIT

#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <cake/base_node.hpp>
#include <cake/context.hpp>
#include <cake/subscriber.hpp>

namespace test_package::backward_compat_node {

template <typename ContextType> struct BackwardCompatNodePublishers {
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr int_qos_pub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr default_qos_pub;
};

template <typename ContextType> struct BackwardCompatNodeSubscribers {
    std::shared_ptr<cake::Subscriber<std_msgs::msg::String, ContextType>> int_qos_sub;
    std::shared_ptr<cake::Subscriber<std_msgs::msg::String, ContextType>> default_qos_sub;
};

template <typename ContextType> struct BackwardCompatNodeServices {};

template <typename ContextType> struct BackwardCompatNodeServiceClients {};

template <typename DerivedContextType> struct BackwardCompatNodeContext : cake::Context {
    BackwardCompatNodePublishers<DerivedContextType> publishers;
    BackwardCompatNodeSubscribers<DerivedContextType> subscribers;
    BackwardCompatNodeServices<DerivedContextType> services;
    BackwardCompatNodeServiceClients<DerivedContextType> service_clients;
};


template <
    typename ContextType,
    auto init_func,
    auto extend_options = [](rclcpp::NodeOptions options) { return options; }>
class BackwardCompatNodeBase : public cake::BaseNode<"backward_compat_node", extend_options> {
  public:
    explicit BackwardCompatNodeBase(const rclcpp::NodeOptions &options) : cake::BaseNode<"backward_compat_node", extend_options>(options) {
        static_assert(
            std::is_base_of_v<BackwardCompatNodeContext<ContextType>, ContextType>, "ContextType must be a child of BackwardCompatNodeContext"
        );

        // init context
        auto ctx = std::make_shared<ContextType>();
        ctx->node = this->node_;

        // init publishers
        ctx->publishers.int_qos_pub = ctx->node->template create_publisher<std_msgs::msg::String>("int_qos_pub", 10);
        ctx->publishers.default_qos_pub = ctx->node->template create_publisher<std_msgs::msg::String>("default_qos_pub", 10);
        // init subscribers
        ctx->subscribers.int_qos_sub = cake::create_subscriber<std_msgs::msg::String>(ctx, "int_qos_sub", 5);
        ctx->subscribers.default_qos_sub = cake::create_subscriber<std_msgs::msg::String>(ctx, "default_qos_sub", 10);
        init_func(ctx);
    }
};

} // namespace test_package::backward_compat_node
