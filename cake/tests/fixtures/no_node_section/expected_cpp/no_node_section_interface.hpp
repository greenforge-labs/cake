// auto-generated DO NOT EDIT

#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <cake/base_node.hpp>
#include <cake/context.hpp>
#include <cake/publisher.hpp>
#include <cake/subscriber.hpp>
#include <test_package/no_node_section_parameters.hpp>

namespace test_package::no_node_section {

template <typename ContextType> struct NoNodeSectionPublishers {
    std::shared_ptr<cake::Publisher<std_msgs::msg::String, ContextType>> status;
};

template <typename ContextType> struct NoNodeSectionSubscribers {
    std::shared_ptr<cake::Subscriber<std_msgs::msg::Bool, ContextType>> input;
};

template <typename ContextType> struct NoNodeSectionServices {};

template <typename ContextType> struct NoNodeSectionServiceClients {};

template <typename ContextType> struct NoNodeSectionActions {};

template <typename ContextType> struct NoNodeSectionActionClients {};

template <typename DerivedContextType> struct NoNodeSectionContext : cake::Context {
    NoNodeSectionPublishers<DerivedContextType> publishers;
    NoNodeSectionSubscribers<DerivedContextType> subscribers;
    NoNodeSectionServices<DerivedContextType> services;
    NoNodeSectionServiceClients<DerivedContextType> service_clients;
    NoNodeSectionActions<DerivedContextType> actions;
    NoNodeSectionActionClients<DerivedContextType> action_clients;
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
class NoNodeSectionBase : public cake::BaseNode<"no_node_section", ContextType, extend_options> {
    static_assert(
        std::is_base_of_v<NoNodeSectionContext<ContextType>, ContextType>, "ContextType must be a child of NoNodeSectionContext"
    );

  public:
    explicit NoNodeSectionBase(const rclcpp::NodeOptions &options)
        : cake::BaseNode<"no_node_section", ContextType, extend_options>(options) {}

  protected:
    void create_entities(std::shared_ptr<ContextType> ctx) override {
        // init parameters (must be before publishers/subscribers for QoS param refs)
        ctx->param_listener = std::make_shared<ParamListener>(ctx->node);
        ctx->params = ctx->param_listener->get_params();

        // init publishers
        ctx->publishers.status = cake::create_publisher<std_msgs::msg::String>(ctx, "/status", rclcpp::QoS(10).reliable());
        // init subscribers
        ctx->subscribers.input = cake::create_subscriber<std_msgs::msg::Bool>(ctx, "/input", rclcpp::QoS(5).best_effort());
    }

    void activate_entities(std::shared_ptr<ContextType> ctx) override {
        ctx->publishers.status->activate();
        for (auto &t : ctx->timers) { t->reset(); }
    }

    void deactivate_entities(std::shared_ptr<ContextType> ctx) override {
        for (auto &t : ctx->timers) { t->cancel(); }
        if (ctx->publishers.status) { ctx->publishers.status->deactivate(); }
    }

    CallbackReturn user_on_configure(std::shared_ptr<ContextType> ctx) override { return on_configure_func(ctx); }
    CallbackReturn user_on_activate(std::shared_ptr<ContextType> ctx) override { return on_activate_func(ctx); }
    CallbackReturn user_on_deactivate(std::shared_ptr<ContextType> ctx) override { return on_deactivate_func(ctx); }
    CallbackReturn user_on_cleanup(std::shared_ptr<ContextType> ctx) override { return on_cleanup_func(ctx); }
    void user_on_shutdown(std::shared_ptr<ContextType> ctx) override { on_shutdown_func(ctx); }
};

} // namespace test_package::no_node_section
