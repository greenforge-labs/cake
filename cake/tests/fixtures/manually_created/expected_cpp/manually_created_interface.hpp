// auto-generated DO NOT EDIT

#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <cake/base_node.hpp>
#include <cake/context.hpp>
#include <cake/publisher.hpp>
#include <cake/subscriber.hpp>
#include <test_package/manually_created_parameters.hpp>

namespace test_package::manually_created {

template <typename ContextType> struct ManuallyCreatedPublishers {
    std::shared_ptr<cake::Publisher<std_msgs::msg::String, ContextType>> auto_topic;
};

template <typename ContextType> struct ManuallyCreatedSubscribers {
    std::shared_ptr<cake::Subscriber<std_msgs::msg::Bool, ContextType>> auto_sub;
};

template <typename ContextType> struct ManuallyCreatedServices {};

template <typename ContextType> struct ManuallyCreatedServiceClients {};

template <typename ContextType> struct ManuallyCreatedActions {};

template <typename ContextType> struct ManuallyCreatedActionClients {};

template <typename DerivedContextType> struct ManuallyCreatedContext : cake::Context {
    ManuallyCreatedPublishers<DerivedContextType> publishers;
    ManuallyCreatedSubscribers<DerivedContextType> subscribers;
    ManuallyCreatedServices<DerivedContextType> services;
    ManuallyCreatedServiceClients<DerivedContextType> service_clients;
    ManuallyCreatedActions<DerivedContextType> actions;
    ManuallyCreatedActionClients<DerivedContextType> action_clients;
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
class ManuallyCreatedBase : public cake::BaseNode<"manually_created", ContextType, extend_options> {
    static_assert(
        std::is_base_of_v<ManuallyCreatedContext<ContextType>, ContextType>, "ContextType must be a child of ManuallyCreatedContext"
    );

  public:
    explicit ManuallyCreatedBase(const rclcpp::NodeOptions &options)
        : cake::BaseNode<"manually_created", ContextType, extend_options>(options) {}

  protected:
    void create_entities(std::shared_ptr<ContextType> ctx) override {
        // init parameters (must be before publishers/subscribers for QoS param refs)
        ctx->param_listener = std::make_shared<ParamListener>(ctx->node);
        ctx->params = ctx->param_listener->get_params();

        // init publishers
        ctx->publishers.auto_topic = cake::create_publisher<std_msgs::msg::String>(ctx, "auto_topic", rclcpp::QoS(10).reliable());
        // init subscribers
        ctx->subscribers.auto_sub = cake::create_subscriber<std_msgs::msg::Bool>(ctx, "auto_sub", rclcpp::QoS(10).best_effort());
    }

    void activate_entities(std::shared_ptr<ContextType> ctx) override {
        ctx->publishers.auto_topic->activate();
        for (auto &t : ctx->timers) { t->reset(); }
    }

    void deactivate_entities(std::shared_ptr<ContextType> ctx) override {
        for (auto &t : ctx->timers) { t->cancel(); }
        if (ctx->publishers.auto_topic) { ctx->publishers.auto_topic->deactivate(); }
    }

    CallbackReturn user_on_configure(std::shared_ptr<ContextType> ctx) override { return on_configure_func(ctx); }
    CallbackReturn user_on_activate(std::shared_ptr<ContextType> ctx) override { return on_activate_func(ctx); }
    CallbackReturn user_on_deactivate(std::shared_ptr<ContextType> ctx) override { return on_deactivate_func(ctx); }
    CallbackReturn user_on_cleanup(std::shared_ptr<ContextType> ctx) override { return on_cleanup_func(ctx); }
    void user_on_shutdown(std::shared_ptr<ContextType> ctx) override { on_shutdown_func(ctx); }
};

} // namespace test_package::manually_created
