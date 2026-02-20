// auto-generated DO NOT EDIT

#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <example_interfaces/srv/add_two_ints.hpp>
#include <cake/base_node.hpp>
#include <cake/context.hpp>
#include <cake/service.hpp>
#include <test_package/services_only_parameters.hpp>

namespace test_package::services_only {

template <typename ContextType> struct ServicesOnlyPublishers {};

template <typename ContextType> struct ServicesOnlySubscribers {};

template <typename ContextType> struct ServicesOnlyServices {
    std::shared_ptr<cake::Service<example_interfaces::srv::AddTwoInts, ContextType>> add_two_ints;
    std::shared_ptr<cake::Service<example_interfaces::srv::AddTwoInts, ContextType>> math_multiply;
};

template <typename ContextType> struct ServicesOnlyServiceClients {};

template <typename ContextType> struct ServicesOnlyActions {};

template <typename ContextType> struct ServicesOnlyActionClients {};

template <typename DerivedContextType> struct ServicesOnlyContext : cake::Context {
    ServicesOnlyPublishers<DerivedContextType> publishers;
    ServicesOnlySubscribers<DerivedContextType> subscribers;
    ServicesOnlyServices<DerivedContextType> services;
    ServicesOnlyServiceClients<DerivedContextType> service_clients;
    ServicesOnlyActions<DerivedContextType> actions;
    ServicesOnlyActionClients<DerivedContextType> action_clients;
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
class ServicesOnlyBase : public cake::BaseNode<"services_only", ContextType, extend_options> {
    static_assert(
        std::is_base_of_v<ServicesOnlyContext<ContextType>, ContextType>, "ContextType must be a child of ServicesOnlyContext"
    );

  public:
    explicit ServicesOnlyBase(const rclcpp::NodeOptions &options)
        : cake::BaseNode<"services_only", ContextType, extend_options>(options) {}

  protected:
    void create_entities(std::shared_ptr<ContextType> ctx) override {
        // init parameters (must be before publishers/subscribers for QoS param refs)
        ctx->param_listener = std::make_shared<ParamListener>(ctx->node);
        ctx->params = ctx->param_listener->get_params();
        // init services
        ctx->services.add_two_ints = cake::create_service<example_interfaces::srv::AddTwoInts>(ctx, "add_two_ints");
        ctx->services.math_multiply = cake::create_service<example_interfaces::srv::AddTwoInts>(ctx, "/math/multiply");
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

} // namespace test_package::services_only
