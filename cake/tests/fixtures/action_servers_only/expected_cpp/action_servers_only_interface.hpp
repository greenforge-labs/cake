// auto-generated DO NOT EDIT

#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/action/fibonacci.hpp>
#include <cake/base_node.hpp>
#include <cake/context.hpp>
#include <cake/action_server.hpp>
#include <test_package/action_servers_only_parameters.hpp>

namespace test_package::action_servers_only {

template <typename ContextType> struct ActionServersOnlyPublishers {};

template <typename ContextType> struct ActionServersOnlySubscribers {};

template <typename ContextType> struct ActionServersOnlyServices {};

template <typename ContextType> struct ActionServersOnlyServiceClients {};

template <typename ContextType> struct ActionServersOnlyActions {
    std::shared_ptr<cake::SingleGoalActionServer<example_interfaces::action::Fibonacci>> fibonacci;
    std::shared_ptr<cake::SingleGoalActionServer<example_interfaces::action::Fibonacci>> math_compute;
};

template <typename ContextType> struct ActionServersOnlyActionClients {};

template <typename DerivedContextType> struct ActionServersOnlyContext : cake::Context {
    ActionServersOnlyPublishers<DerivedContextType> publishers;
    ActionServersOnlySubscribers<DerivedContextType> subscribers;
    ActionServersOnlyServices<DerivedContextType> services;
    ActionServersOnlyServiceClients<DerivedContextType> service_clients;
    ActionServersOnlyActions<DerivedContextType> actions;
    ActionServersOnlyActionClients<DerivedContextType> action_clients;
    std::shared_ptr<ParamListener> param_listener;
    Params params;
};


template <
    typename ContextType,
    auto init_func,
    auto extend_options = [](rclcpp::NodeOptions options) { return options; }>
class ActionServersOnlyBase : public cake::BaseNode<"action_servers_only", extend_options> {
  public:
    explicit ActionServersOnlyBase(const rclcpp::NodeOptions &options) : cake::BaseNode<"action_servers_only", extend_options>(options) {
        static_assert(
            std::is_base_of_v<ActionServersOnlyContext<ContextType>, ContextType>, "ContextType must be a child of ActionServersOnlyContext"
        );

        // init context
        auto ctx = std::make_shared<ContextType>();
        ctx->node = this->node_;

        // init parameters (must be before publishers/subscribers for QoS param refs)
        ctx->param_listener = std::make_shared<ParamListener>(ctx->node);
        ctx->params = ctx->param_listener->get_params();
        // init actions
        ctx->actions.fibonacci = cake::create_single_goal_action_server<example_interfaces::action::Fibonacci>(ctx, "fibonacci");
        ctx->actions.math_compute = cake::create_single_goal_action_server<example_interfaces::action::Fibonacci>(ctx, "/math/compute");
        init_func(ctx);
    }
};

} // namespace test_package::action_servers_only
