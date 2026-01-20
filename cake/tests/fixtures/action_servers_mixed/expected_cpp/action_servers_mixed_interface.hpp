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
#include <cake/publisher.hpp>
#include <cake/subscriber.hpp>
#include <cake/service.hpp>
#include <cake/action_server.hpp>
#include <test_package/action_servers_mixed_parameters.hpp>

namespace test_package::action_servers_mixed {

template <typename ContextType> struct ActionServersMixedPublishers {
    std::shared_ptr<cake::Publisher<std_msgs::msg::String, ContextType>> status;
};

template <typename ContextType> struct ActionServersMixedSubscribers {
    std::shared_ptr<cake::Subscriber<std_msgs::msg::Bool, ContextType>> cmd;
};

template <typename ContextType> struct ActionServersMixedServices {
    std::shared_ptr<cake::Service<std_srvs::srv::Trigger, ContextType>> reset;
};

template <typename ContextType> struct ActionServersMixedServiceClients {};

template <typename ContextType> struct ActionServersMixedActions {
    std::shared_ptr<cake::SingleGoalActionServer<example_interfaces::action::Fibonacci>> navigate;
};

template <typename ContextType> struct ActionServersMixedActionClients {};

template <typename DerivedContextType> struct ActionServersMixedContext : cake::Context {
    ActionServersMixedPublishers<DerivedContextType> publishers;
    ActionServersMixedSubscribers<DerivedContextType> subscribers;
    ActionServersMixedServices<DerivedContextType> services;
    ActionServersMixedServiceClients<DerivedContextType> service_clients;
    ActionServersMixedActions<DerivedContextType> actions;
    ActionServersMixedActionClients<DerivedContextType> action_clients;
    std::shared_ptr<ParamListener> param_listener;
    Params params;
};


template <
    typename ContextType,
    auto init_func,
    auto extend_options = [](rclcpp::NodeOptions options) { return options; }>
class ActionServersMixedBase : public cake::BaseNode<"action_servers_mixed", extend_options> {
  public:
    explicit ActionServersMixedBase(const rclcpp::NodeOptions &options) : cake::BaseNode<"action_servers_mixed", extend_options>(options) {
        static_assert(
            std::is_base_of_v<ActionServersMixedContext<ContextType>, ContextType>, "ContextType must be a child of ActionServersMixedContext"
        );

        // init context
        auto ctx = std::make_shared<ContextType>();
        ctx->node = this->node_;

        // init publishers
        ctx->publishers.status = cake::create_publisher<std_msgs::msg::String>(ctx, "/status", rclcpp::QoS(10).reliable());
        // init subscribers
        ctx->subscribers.cmd = cake::create_subscriber<std_msgs::msg::Bool>(ctx, "/cmd", rclcpp::QoS(10).best_effort());
        // init services
        ctx->services.reset = cake::create_service<std_srvs::srv::Trigger>(ctx, "/reset");
        // init actions
        ctx->actions.navigate = cake::create_single_goal_action_server<example_interfaces::action::Fibonacci>(ctx, "navigate");
        // init parameters
        ctx->param_listener = std::make_shared<ParamListener>(ctx->node);
        ctx->params = ctx->param_listener->get_params();

        init_func(ctx);
    }
};

} // namespace test_package::action_servers_mixed
