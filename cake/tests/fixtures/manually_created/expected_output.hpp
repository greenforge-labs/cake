// auto-generated DO NOT EDIT

#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <cake/base_node.hpp>
#include <cake/context.hpp>
#include <cake/subscriber.hpp>
#include <test_package/manually_created_parameters.hpp>

namespace test_package::manually_created {

template <typename ContextType> struct ManuallyCreatedPublishers {
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr auto_topic;
};

template <typename ContextType> struct ManuallyCreatedSubscribers {
    std::shared_ptr<cake::Subscriber<std_msgs::msg::Bool, ContextType>> auto_sub;
};

template <typename ContextType> struct ManuallyCreatedServices {};

template <typename ContextType> struct ManuallyCreatedServiceClients {};

template <typename ContextType> struct ManuallyCreatedActions {};

template <typename ContextType> struct ManuallyCreatedActionClients {};

template <typename DerivedContextType> struct ManuallyCreatedContext : cake::Context {
    ManuallyCreatedPublishers<DerivedContextType> publishers;
    ManuallyCreatedSubscribers<DerivedContextType> subscribers;
    ManuallyCreatedServices<DerivedContextType> services;
    ManuallyCreatedServiceClients<DerivedContextType> service_clients;
    ManuallyCreatedActions<DerivedContextType> actions;
    ManuallyCreatedActionClients<DerivedContextType> action_clients;
    std::shared_ptr<ParamListener> param_listener;
    Params params;
};


template <
    typename ContextType,
    auto init_func,
    auto extend_options = [](rclcpp::NodeOptions options) { return options; }>
class ManuallyCreatedBase : public cake::BaseNode<"manually_created", extend_options> {
  public:
    explicit ManuallyCreatedBase(const rclcpp::NodeOptions &options) : cake::BaseNode<"manually_created", extend_options>(options) {
        static_assert(
            std::is_base_of_v<ManuallyCreatedContext<ContextType>, ContextType>, "ContextType must be a child of ManuallyCreatedContext"
        );

        // init context
        auto ctx = std::make_shared<ContextType>();
        ctx->node = this->node_;

        // init publishers
        ctx->publishers.auto_topic = ctx->node->template create_publisher<std_msgs::msg::String>("auto_topic", 10);
        // init subscribers
        ctx->subscribers.auto_sub = cake::create_subscriber<std_msgs::msg::Bool>(ctx, "auto_sub", 10);
        // init parameters
        ctx->param_listener = std::make_shared<ParamListener>(ctx->node);
        ctx->params = ctx->param_listener->get_params();

        init_func(ctx);
    }
};

} // namespace test_package::manually_created
