// auto-generated DO NOT EDIT

#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <cake/base_node.hpp>
#include <cake/context.hpp>
#include <cake/publisher.hpp>
#include <cake/subscriber.hpp>
#include <cake/service.hpp>
#include <test_package/name_param_substitution_parameters.hpp>

namespace test_package::name_param_substitution {

template <typename ContextType> struct NameParamSubstitutionPublishers {
    std::shared_ptr<cake::Publisher<geometry_msgs::msg::Twist, ContextType>> cmd_vel;
};

template <typename ContextType> struct NameParamSubstitutionSubscribers {
    std::shared_ptr<cake::Subscriber<nav_msgs::msg::Odometry, ContextType>> odom;
};

template <typename ContextType> struct NameParamSubstitutionServices {
    std::shared_ptr<cake::Service<std_srvs::srv::Trigger, ContextType>> get_state;
};

template <typename ContextType> struct NameParamSubstitutionServiceClients {
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr external_service;
};

template <typename ContextType> struct NameParamSubstitutionActions {};

template <typename ContextType> struct NameParamSubstitutionActionClients {};

template <typename DerivedContextType> struct NameParamSubstitutionContext : cake::Context {
    NameParamSubstitutionPublishers<DerivedContextType> publishers;
    NameParamSubstitutionSubscribers<DerivedContextType> subscribers;
    NameParamSubstitutionServices<DerivedContextType> services;
    NameParamSubstitutionServiceClients<DerivedContextType> service_clients;
    NameParamSubstitutionActions<DerivedContextType> actions;
    NameParamSubstitutionActionClients<DerivedContextType> action_clients;
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
class NameParamSubstitutionBase : public cake::BaseNode<"name_param_substitution", ContextType, extend_options> {
    static_assert(
        std::is_base_of_v<NameParamSubstitutionContext<ContextType>, ContextType>, "ContextType must be a child of NameParamSubstitutionContext"
    );

  public:
    explicit NameParamSubstitutionBase(const rclcpp::NodeOptions &options)
        : cake::BaseNode<"name_param_substitution", ContextType, extend_options>(options) {}

  protected:
    void create_entities(std::shared_ptr<ContextType> ctx) override {
        // init parameters (must be before publishers/subscribers for QoS param refs)
        ctx->param_listener = std::make_shared<ParamListener>(ctx->node);
        ctx->params = ctx->param_listener->get_params();

        // init publishers
        ctx->publishers.cmd_vel = cake::create_publisher<geometry_msgs::msg::Twist>(ctx, "/robot/" + ctx->params.robot_id + "/cmd_vel", rclcpp::QoS(10).reliable());
        // init subscribers
        ctx->subscribers.odom = cake::create_subscriber<nav_msgs::msg::Odometry>(ctx, "/" + ctx->params.namespace + "/" + ctx->params.robot_id + "/odom", rclcpp::QoS(5).best_effort());
        // init services
        ctx->services.get_state = cake::create_service<std_srvs::srv::Trigger>(ctx, "/robot/" + ctx->params.robot_id + "/get_state");
        // init service clients
        ctx->service_clients.external_service = ctx->node->template create_client<std_srvs::srv::SetBool>("/" + ctx->params.namespace + "/service");
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

} // namespace test_package::name_param_substitution
