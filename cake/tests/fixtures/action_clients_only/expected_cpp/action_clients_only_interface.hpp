// auto-generated DO NOT EDIT

#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/action/fibonacci.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <cake/base_node.hpp>
#include <cake/context.hpp>
#include <test_package/action_clients_only_parameters.hpp>

namespace test_package::action_clients_only {

template <typename ContextType> struct ActionClientsOnlyPublishers {};

template <typename ContextType> struct ActionClientsOnlySubscribers {};

template <typename ContextType> struct ActionClientsOnlyServices {};

template <typename ContextType> struct ActionClientsOnlyServiceClients {};

template <typename ContextType> struct ActionClientsOnlyActions {};

template <typename ContextType> struct ActionClientsOnlyActionClients {
    rclcpp_action::Client<example_interfaces::action::Fibonacci>::SharedPtr fibonacci;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigate_to_pose;
};

template <typename DerivedContextType> struct ActionClientsOnlyContext : cake::Context {
    ActionClientsOnlyPublishers<DerivedContextType> publishers;
    ActionClientsOnlySubscribers<DerivedContextType> subscribers;
    ActionClientsOnlyServices<DerivedContextType> services;
    ActionClientsOnlyServiceClients<DerivedContextType> service_clients;
    ActionClientsOnlyActions<DerivedContextType> actions;
    ActionClientsOnlyActionClients<DerivedContextType> action_clients;
    std::shared_ptr<ParamListener> param_listener;
    Params params;
};


template <
    typename ContextType,
    auto init_func,
    auto extend_options = [](rclcpp::NodeOptions options) { return options; }>
class ActionClientsOnlyBase : public cake::BaseNode<"action_clients_only", extend_options> {
  public:
    explicit ActionClientsOnlyBase(const rclcpp::NodeOptions &options) : cake::BaseNode<"action_clients_only", extend_options>(options) {
        static_assert(
            std::is_base_of_v<ActionClientsOnlyContext<ContextType>, ContextType>, "ContextType must be a child of ActionClientsOnlyContext"
        );

        // init context
        auto ctx = std::make_shared<ContextType>();
        ctx->node = this->node_;
        // init action clients
        ctx->action_clients.fibonacci = rclcpp_action::create_client<example_interfaces::action::Fibonacci>(ctx->node, "/fibonacci");
        ctx->action_clients.navigate_to_pose = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(ctx->node, "navigate_to_pose");
        // init parameters
        ctx->param_listener = std::make_shared<ParamListener>(ctx->node);
        ctx->params = ctx->param_listener->get_params();

        init_func(ctx);
    }
};

} // namespace test_package::action_clients_only
