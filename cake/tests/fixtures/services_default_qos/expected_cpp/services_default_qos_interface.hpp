// auto-generated DO NOT EDIT

#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <example_interfaces/srv/add_two_ints.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <cake/base_node.hpp>
#include <cake/context.hpp>
#include <cake/service.hpp>
#include <test_package/services_default_qos_parameters.hpp>

namespace test_package::services_default_qos {

template <typename ContextType> struct ServicesDefaultQosPublishers {};

template <typename ContextType> struct ServicesDefaultQosSubscribers {};

template <typename ContextType> struct ServicesDefaultQosServices {
    std::shared_ptr<cake::Service<std_srvs::srv::Trigger, ContextType>> trigger_service;
    std::shared_ptr<cake::Service<example_interfaces::srv::AddTwoInts, ContextType>> compute;
};

template <typename ContextType> struct ServicesDefaultQosServiceClients {};

template <typename ContextType> struct ServicesDefaultQosActions {};

template <typename ContextType> struct ServicesDefaultQosActionClients {};

template <typename DerivedContextType> struct ServicesDefaultQosContext : cake::Context {
    ServicesDefaultQosPublishers<DerivedContextType> publishers;
    ServicesDefaultQosSubscribers<DerivedContextType> subscribers;
    ServicesDefaultQosServices<DerivedContextType> services;
    ServicesDefaultQosServiceClients<DerivedContextType> service_clients;
    ServicesDefaultQosActions<DerivedContextType> actions;
    ServicesDefaultQosActionClients<DerivedContextType> action_clients;
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
class ServicesDefaultQosBase : public cake::BaseNode<"services_default_qos", ContextType, extend_options> {
    static_assert(
        std::is_base_of_v<ServicesDefaultQosContext<ContextType>, ContextType>, "ContextType must be a child of ServicesDefaultQosContext"
    );

  public:
    explicit ServicesDefaultQosBase(const rclcpp::NodeOptions &options)
        : cake::BaseNode<"services_default_qos", ContextType, extend_options>(options) {}

  protected:
    void create_entities(std::shared_ptr<ContextType> ctx) override {
        // init parameters (must be before publishers/subscribers for QoS param refs)
        ctx->param_listener = std::make_shared<ParamListener>(ctx->node);
        ctx->params = ctx->param_listener->get_params();
        // init services
        ctx->services.trigger_service = cake::create_service<std_srvs::srv::Trigger>(ctx, "/trigger_service");
        ctx->services.compute = cake::create_service<example_interfaces::srv::AddTwoInts>(ctx, "compute");
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

} // namespace test_package::services_default_qos
