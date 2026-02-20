// auto-generated DO NOT EDIT

#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <cake/base_node.hpp>
#include <cake/context.hpp>
#include <cake/publisher.hpp>
#include <cake/subscriber.hpp>
#include <string>
#include <unordered_map>
#include <test_package/for_each_param_parameters.hpp>

namespace test_package::for_each_param {

template <typename ContextType> struct ForEachParamPublishers {
    std::shared_ptr<cake::Publisher<std_msgs::msg::String, ContextType>> status;
};

template <typename ContextType> struct ForEachParamSubscribers {
    std::unordered_map<std::string, std::shared_ptr<cake::Subscriber<std_msgs::msg::String, ContextType>>> node_states;
};

template <typename ContextType> struct ForEachParamServices {};

template <typename ContextType> struct ForEachParamServiceClients {
    std::unordered_map<std::string, rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr> change_state_clients;
};

template <typename ContextType> struct ForEachParamActions {};

template <typename ContextType> struct ForEachParamActionClients {};

template <typename DerivedContextType> struct ForEachParamContext : cake::Context {
    ForEachParamPublishers<DerivedContextType> publishers;
    ForEachParamSubscribers<DerivedContextType> subscribers;
    ForEachParamServices<DerivedContextType> services;
    ForEachParamServiceClients<DerivedContextType> service_clients;
    ForEachParamActions<DerivedContextType> actions;
    ForEachParamActionClients<DerivedContextType> action_clients;
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
class ForEachParamBase : public cake::BaseNode<"for_each_param", ContextType, extend_options> {
    static_assert(
        std::is_base_of_v<ForEachParamContext<ContextType>, ContextType>, "ContextType must be a child of ForEachParamContext"
    );

  public:
    explicit ForEachParamBase(const rclcpp::NodeOptions &options)
        : cake::BaseNode<"for_each_param", ContextType, extend_options>(options) {}

  protected:
    void create_entities(std::shared_ptr<ContextType> ctx) override {
        // init parameters (must be before publishers/subscribers for QoS param refs)
        ctx->param_listener = std::make_shared<ParamListener>(ctx->node);
        ctx->params = ctx->param_listener->get_params();

        // init publishers
        ctx->publishers.status = cake::create_publisher<std_msgs::msg::String>(ctx, "/robot/" + ctx->params.robot_id + "/status", rclcpp::QoS(10).reliable());
        // init subscribers
        for (const auto& key : ctx->params.managed_nodes) {
            ctx->subscribers.node_states[key] = cake::create_subscriber<std_msgs::msg::String>(ctx, "/" + key + "/state", rclcpp::QoS(10).reliable());
        }
        // init service clients
        for (const auto& key : ctx->params.managed_nodes) {
            ctx->service_clients.change_state_clients[key] = ctx->node->template create_client<lifecycle_msgs::srv::ChangeState>("/" + key + "/change_state");
        }
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

} // namespace test_package::for_each_param
