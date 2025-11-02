// auto-generated DO NOT EDIT

#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/action/fibonacci.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <cake/base_node.hpp>
#include <cake/context.hpp>
#include <cake/subscriber.hpp>
#include <cake/service.hpp>
#include <cake/action_server.hpp>

namespace test_package::mixed_node {

template <typename ContextType> struct MixedNodePublishers {
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status;
};

template <typename ContextType> struct MixedNodeSubscribers {
    std::shared_ptr<cake::Subscriber<std_msgs::msg::Bool, ContextType>> cmd;
};

template <typename ContextType> struct MixedNodeServices {
    std::shared_ptr<cake::Service<std_srvs::srv::Trigger, ContextType>> reset;
};

template <typename ContextType> struct MixedNodeServiceClients {};

template <typename ContextType> struct MixedNodeActions {
    std::shared_ptr<cake::SingleGoalActionServer<example_interfaces::action::Fibonacci>> navigate;
};

template <typename DerivedContextType> struct MixedNodeContext : cake::Context {
    MixedNodePublishers<DerivedContextType> publishers;
    MixedNodeSubscribers<DerivedContextType> subscribers;
    MixedNodeServices<DerivedContextType> services;
    MixedNodeServiceClients<DerivedContextType> service_clients;
    MixedNodeActions<DerivedContextType> actions;
};


template <
    typename ContextType,
    auto init_func,
    auto extend_options = [](rclcpp::NodeOptions options) { return options; }>
class MixedNodeBase : public cake::BaseNode<"mixed_node", extend_options> {
  public:
    explicit MixedNodeBase(const rclcpp::NodeOptions &options) : cake::BaseNode<"mixed_node", extend_options>(options) {
        static_assert(
            std::is_base_of_v<MixedNodeContext<ContextType>, ContextType>, "ContextType must be a child of MixedNodeContext"
        );

        // init context
        auto ctx = std::make_shared<ContextType>();
        ctx->node = this->node_;

        // init publishers
        ctx->publishers.status = ctx->node->template create_publisher<std_msgs::msg::String>("/status", 10);
        // init subscribers
        ctx->subscribers.cmd = cake::create_subscriber<std_msgs::msg::Bool>(ctx, "/cmd", 10);
        // init services
        ctx->services.reset = cake::create_service<std_srvs::srv::Trigger>(ctx, "/reset");
        // init actions
        ctx->actions.navigate = cake::create_single_goal_action_server<example_interfaces::action::Fibonacci>(ctx, "navigate");
        init_func(ctx);
    }
};

} // namespace test_package::mixed_node
