// auto-generated DO NOT EDIT

#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <cake/base_node.hpp>
#include <cake/context.hpp>
#include <test_package/empty_node_parameters.hpp>

namespace test_package::empty_node {

template <typename ContextType> struct EmptyNodePublishers {};

template <typename ContextType> struct EmptyNodeSubscribers {};

template <typename ContextType> struct EmptyNodeServices {};

template <typename ContextType> struct EmptyNodeServiceClients {};

template <typename ContextType> struct EmptyNodeActions {};

template <typename ContextType> struct EmptyNodeActionClients {};

template <typename DerivedContextType> struct EmptyNodeContext : cake::Context {
    EmptyNodePublishers<DerivedContextType> publishers;
    EmptyNodeSubscribers<DerivedContextType> subscribers;
    EmptyNodeServices<DerivedContextType> services;
    EmptyNodeServiceClients<DerivedContextType> service_clients;
    EmptyNodeActions<DerivedContextType> actions;
    EmptyNodeActionClients<DerivedContextType> action_clients;
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
class EmptyNodeBase : public cake::BaseNode<"empty_node", ContextType, extend_options> {
    static_assert(
        std::is_base_of_v<EmptyNodeContext<ContextType>, ContextType>, "ContextType must be a child of EmptyNodeContext"
    );

  public:
    explicit EmptyNodeBase(const rclcpp::NodeOptions &options)
        : cake::BaseNode<"empty_node", ContextType, extend_options>(options) {}

  protected:
    void create_entities(std::shared_ptr<ContextType> ctx) override {
        // init parameters (must be before publishers/subscribers for QoS param refs)
        ctx->param_listener = std::make_shared<ParamListener>(ctx->node);
        ctx->params = ctx->param_listener->get_params();
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

} // namespace test_package::empty_node
