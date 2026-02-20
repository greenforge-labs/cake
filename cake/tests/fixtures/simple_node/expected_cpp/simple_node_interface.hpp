// auto-generated DO NOT EDIT

#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cake/base_node.hpp>
#include <cake/context.hpp>
#include <cake/publisher.hpp>
#include <cake/subscriber.hpp>
#include <test_package/simple_node_parameters.hpp>

namespace test_package::simple_node {

template <typename ContextType> struct SimpleNodePublishers {
    std::shared_ptr<cake::Publisher<geometry_msgs::msg::Twist, ContextType>> cmd_vel;
};

template <typename ContextType> struct SimpleNodeSubscribers {
    std::shared_ptr<cake::Subscriber<nav_msgs::msg::Odometry, ContextType>> odom;
};

template <typename ContextType> struct SimpleNodeServices {};

template <typename ContextType> struct SimpleNodeServiceClients {};

template <typename ContextType> struct SimpleNodeActions {};

template <typename ContextType> struct SimpleNodeActionClients {};

template <typename DerivedContextType> struct SimpleNodeContext : cake::Context {
    SimpleNodePublishers<DerivedContextType> publishers;
    SimpleNodeSubscribers<DerivedContextType> subscribers;
    SimpleNodeServices<DerivedContextType> services;
    SimpleNodeServiceClients<DerivedContextType> service_clients;
    SimpleNodeActions<DerivedContextType> actions;
    SimpleNodeActionClients<DerivedContextType> action_clients;
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
class SimpleNodeBase : public cake::BaseNode<"simple_node", ContextType, extend_options> {
    static_assert(
        std::is_base_of_v<SimpleNodeContext<ContextType>, ContextType>, "ContextType must be a child of SimpleNodeContext"
    );

  public:
    explicit SimpleNodeBase(const rclcpp::NodeOptions &options)
        : cake::BaseNode<"simple_node", ContextType, extend_options>(options) {}

  protected:
    void create_entities(std::shared_ptr<ContextType> ctx) override {
        // init parameters (must be before publishers/subscribers for QoS param refs)
        ctx->param_listener = std::make_shared<ParamListener>(ctx->node);
        ctx->params = ctx->param_listener->get_params();

        // init publishers
        ctx->publishers.cmd_vel = cake::create_publisher<geometry_msgs::msg::Twist>(ctx, "/cmd_vel", rclcpp::QoS(10).reliable());
        // init subscribers
        ctx->subscribers.odom = cake::create_subscriber<nav_msgs::msg::Odometry>(ctx, "/odom", rclcpp::QoS(10).reliable());
    }

    void activate_entities(std::shared_ptr<ContextType> ctx) override {
        ctx->publishers.cmd_vel->activate();
        for (auto &t : ctx->timers) { t->reset(); }
    }

    void deactivate_entities(std::shared_ptr<ContextType> ctx) override {
        for (auto &t : ctx->timers) { t->cancel(); }
        if (ctx->publishers.cmd_vel) { ctx->publishers.cmd_vel->deactivate(); }
    }

    CallbackReturn user_on_configure(std::shared_ptr<ContextType> ctx) override { return on_configure_func(ctx); }
    CallbackReturn user_on_activate(std::shared_ptr<ContextType> ctx) override { return on_activate_func(ctx); }
    CallbackReturn user_on_deactivate(std::shared_ptr<ContextType> ctx) override { return on_deactivate_func(ctx); }
    CallbackReturn user_on_cleanup(std::shared_ptr<ContextType> ctx) override { return on_cleanup_func(ctx); }
    void user_on_shutdown(std::shared_ptr<ContextType> ctx) override { on_shutdown_func(ctx); }
};

} // namespace test_package::simple_node
