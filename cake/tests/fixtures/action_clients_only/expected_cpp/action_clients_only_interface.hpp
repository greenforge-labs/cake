// auto-generated DO NOT EDIT

#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
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

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

template <
    typename ContextType,
    auto on_configure_func,
    auto on_activate_func = [](std::shared_ptr<ContextType>) { return CallbackReturn::SUCCESS; },
    auto on_deactivate_func = [](std::shared_ptr<ContextType>) { return CallbackReturn::SUCCESS; },
    auto on_cleanup_func = [](std::shared_ptr<ContextType>) { return CallbackReturn::SUCCESS; },
    auto on_shutdown_func = [](std::shared_ptr<ContextType>) {},
    auto extend_options = [](rclcpp::NodeOptions options) { return options; }>
class ActionClientsOnlyBase : public cake::BaseNode<"action_clients_only", ContextType, extend_options> {
    static_assert(
        std::is_base_of_v<ActionClientsOnlyContext<ContextType>, ContextType>, "ContextType must be a child of ActionClientsOnlyContext"
    );

  public:
    explicit ActionClientsOnlyBase(const rclcpp::NodeOptions &options)
        : cake::BaseNode<"action_clients_only", ContextType, extend_options>(options) {}

  protected:
    void create_entities(std::shared_ptr<ContextType> ctx) override {
        // init parameters (must be before publishers/subscribers for QoS param refs)
        ctx->param_listener = std::make_shared<ParamListener>(ctx->node);
        ctx->params = ctx->param_listener->get_params();
        // init action clients
        ctx->action_clients.fibonacci = rclcpp_action::create_client<example_interfaces::action::Fibonacci>(ctx->node, "/fibonacci");
        ctx->action_clients.navigate_to_pose = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(ctx->node, "navigate_to_pose");
    }

    void activate_entities(std::shared_ptr<ContextType> ctx) override {
        for (auto &t : ctx->timers) { t->reset(); }
    }

    void deactivate_entities(std::shared_ptr<ContextType> ctx) override {
        for (auto &t : ctx->timers) { t->cancel(); }
    }

    CallbackReturn user_on_configure(std::shared_ptr<ContextType> ctx) override { return on_configure_func(ctx); }
    CallbackReturn user_on_activate(std::shared_ptr<ContextType> ctx) override { return on_activate_func(ctx); }
    CallbackReturn user_on_deactivate(std::shared_ptr<ContextType> ctx) override { return on_deactivate_func(ctx); }
    CallbackReturn user_on_cleanup(std::shared_ptr<ContextType> ctx) override { return on_cleanup_func(ctx); }
    void user_on_shutdown(std::shared_ptr<ContextType> ctx) override { on_shutdown_func(ctx); }
};

} // namespace test_package::action_clients_only
