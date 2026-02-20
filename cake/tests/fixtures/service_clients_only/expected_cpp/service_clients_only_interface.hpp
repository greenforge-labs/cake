// auto-generated DO NOT EDIT

#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <example_interfaces/srv/add_two_ints.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <cake/base_node.hpp>
#include <cake/context.hpp>
#include <test_package/service_clients_only_parameters.hpp>

namespace test_package::service_clients_only {

template <typename ContextType> struct ServiceClientsOnlyPublishers {};

template <typename ContextType> struct ServiceClientsOnlySubscribers {};

template <typename ContextType> struct ServiceClientsOnlyServices {};

template <typename ContextType> struct ServiceClientsOnlyServiceClients {
    rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr add_two_ints;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr trigger_service;
};

template <typename ContextType> struct ServiceClientsOnlyActions {};

template <typename ContextType> struct ServiceClientsOnlyActionClients {};

template <typename DerivedContextType> struct ServiceClientsOnlyContext : cake::Context {
    ServiceClientsOnlyPublishers<DerivedContextType> publishers;
    ServiceClientsOnlySubscribers<DerivedContextType> subscribers;
    ServiceClientsOnlyServices<DerivedContextType> services;
    ServiceClientsOnlyServiceClients<DerivedContextType> service_clients;
    ServiceClientsOnlyActions<DerivedContextType> actions;
    ServiceClientsOnlyActionClients<DerivedContextType> action_clients;
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
class ServiceClientsOnlyBase : public cake::BaseNode<"service_clients_only", ContextType, extend_options> {
    static_assert(
        std::is_base_of_v<ServiceClientsOnlyContext<ContextType>, ContextType>, "ContextType must be a child of ServiceClientsOnlyContext"
    );

  public:
    explicit ServiceClientsOnlyBase(const rclcpp::NodeOptions &options)
        : cake::BaseNode<"service_clients_only", ContextType, extend_options>(options) {}

  protected:
    void create_entities(std::shared_ptr<ContextType> ctx) override {
        // init parameters (must be before publishers/subscribers for QoS param refs)
        ctx->param_listener = std::make_shared<ParamListener>(ctx->node);
        ctx->params = ctx->param_listener->get_params();
        // init service clients
        ctx->service_clients.add_two_ints = ctx->node->template create_client<example_interfaces::srv::AddTwoInts>("/add_two_ints");
        ctx->service_clients.trigger_service = ctx->node->template create_client<std_srvs::srv::Trigger>("trigger_service");
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

} // namespace test_package::service_clients_only
