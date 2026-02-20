// auto-generated DO NOT EDIT

#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <cake/base_node.hpp>
#include <cake/context.hpp>
#include <cake/publisher.hpp>
#include <test_package/publishers_only_parameters.hpp>

namespace test_package::publishers_only {

template <typename ContextType> struct PublishersOnlyPublishers {
    std::shared_ptr<cake::Publisher<std_msgs::msg::String, ContextType>> status;
    std::shared_ptr<cake::Publisher<std_msgs::msg::Int32, ContextType>> counter;
};

template <typename ContextType> struct PublishersOnlySubscribers {};

template <typename ContextType> struct PublishersOnlyServices {};

template <typename ContextType> struct PublishersOnlyServiceClients {};

template <typename ContextType> struct PublishersOnlyActions {};

template <typename ContextType> struct PublishersOnlyActionClients {};

template <typename DerivedContextType> struct PublishersOnlyContext : cake::Context {
    PublishersOnlyPublishers<DerivedContextType> publishers;
    PublishersOnlySubscribers<DerivedContextType> subscribers;
    PublishersOnlyServices<DerivedContextType> services;
    PublishersOnlyServiceClients<DerivedContextType> service_clients;
    PublishersOnlyActions<DerivedContextType> actions;
    PublishersOnlyActionClients<DerivedContextType> action_clients;
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
class PublishersOnlyBase : public cake::BaseNode<"publishers_only", ContextType, extend_options> {
    static_assert(
        std::is_base_of_v<PublishersOnlyContext<ContextType>, ContextType>, "ContextType must be a child of PublishersOnlyContext"
    );

  public:
    explicit PublishersOnlyBase(const rclcpp::NodeOptions &options)
        : cake::BaseNode<"publishers_only", ContextType, extend_options>(options) {}

  protected:
    void create_entities(std::shared_ptr<ContextType> ctx) override {
        // init parameters (must be before publishers/subscribers for QoS param refs)
        ctx->param_listener = std::make_shared<ParamListener>(ctx->node);
        ctx->params = ctx->param_listener->get_params();

        // init publishers
        ctx->publishers.status = cake::create_publisher<std_msgs::msg::String>(ctx, "status", rclcpp::QoS(10).reliable());
        ctx->publishers.counter = cake::create_publisher<std_msgs::msg::Int32>(ctx, "counter", rclcpp::QoS(5).best_effort());
    }

    void activate_entities(std::shared_ptr<ContextType> ctx) override {
        ctx->publishers.status->activate();
        ctx->publishers.counter->activate();
        for (auto &t : ctx->timers) { t->reset(); }
    }

    void deactivate_entities(std::shared_ptr<ContextType> ctx) override {
        for (auto &t : ctx->timers) { t->cancel(); }
        if (ctx->publishers.status) { ctx->publishers.status->deactivate(); }
        if (ctx->publishers.counter) { ctx->publishers.counter->deactivate(); }
    }

    CallbackReturn user_on_configure(std::shared_ptr<ContextType> ctx) override { return on_configure_func(ctx); }
    CallbackReturn user_on_activate(std::shared_ptr<ContextType> ctx) override { return on_activate_func(ctx); }
    CallbackReturn user_on_deactivate(std::shared_ptr<ContextType> ctx) override { return on_deactivate_func(ctx); }
    CallbackReturn user_on_cleanup(std::shared_ptr<ContextType> ctx) override { return on_cleanup_func(ctx); }
    void user_on_shutdown(std::shared_ptr<ContextType> ctx) override { on_shutdown_func(ctx); }
};

} // namespace test_package::publishers_only
