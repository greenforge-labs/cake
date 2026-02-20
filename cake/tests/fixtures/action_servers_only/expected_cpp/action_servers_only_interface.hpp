// auto-generated DO NOT EDIT

#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
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

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

template <
    typename ContextType,
    auto on_configure_func,
    auto on_activate_func = [](std::shared_ptr<ContextType>) { return CallbackReturn::SUCCESS; },
    auto on_deactivate_func = [](std::shared_ptr<ContextType>) { return CallbackReturn::SUCCESS; },
    auto on_cleanup_func = [](std::shared_ptr<ContextType>) { return CallbackReturn::SUCCESS; },
    auto on_shutdown_func = [](std::shared_ptr<ContextType>) {},
    auto extend_options = [](rclcpp::NodeOptions options) { return options; }>
class ActionServersOnlyBase : public cake::BaseNode<"action_servers_only", ContextType, extend_options> {
    static_assert(
        std::is_base_of_v<ActionServersOnlyContext<ContextType>, ContextType>, "ContextType must be a child of ActionServersOnlyContext"
    );

  public:
    explicit ActionServersOnlyBase(const rclcpp::NodeOptions &options)
        : cake::BaseNode<"action_servers_only", ContextType, extend_options>(options) {}

  protected:
    void create_entities(std::shared_ptr<ContextType> ctx) override {
        // init parameters (must be before publishers/subscribers for QoS param refs)
        ctx->param_listener = std::make_shared<ParamListener>(ctx->node);
        ctx->params = ctx->param_listener->get_params();
        // init actions
        ctx->actions.fibonacci = cake::create_single_goal_action_server<example_interfaces::action::Fibonacci>(ctx, "fibonacci");
        ctx->actions.math_compute = cake::create_single_goal_action_server<example_interfaces::action::Fibonacci>(ctx, "/math/compute");
    }

    void activate_entities(std::shared_ptr<ContextType> ctx) override {
        for (auto &t : ctx->timers) { t->reset(); }
    }

    void deactivate_entities(std::shared_ptr<ContextType> ctx) override {
        for (auto &t : ctx->timers) { t->cancel(); }
        if (ctx->actions.fibonacci) { ctx->actions.fibonacci->deactivate(); }
        if (ctx->actions.math_compute) { ctx->actions.math_compute->deactivate(); }
    }

    CallbackReturn user_on_configure(std::shared_ptr<ContextType> ctx) override { return on_configure_func(ctx); }
    CallbackReturn user_on_activate(std::shared_ptr<ContextType> ctx) override { return on_activate_func(ctx); }
    CallbackReturn user_on_deactivate(std::shared_ptr<ContextType> ctx) override { return on_deactivate_func(ctx); }
    CallbackReturn user_on_cleanup(std::shared_ptr<ContextType> ctx) override { return on_cleanup_func(ctx); }
    void user_on_shutdown(std::shared_ptr<ContextType> ctx) override { on_shutdown_func(ctx); }
};

} // namespace test_package::action_servers_only
