// auto-generated DO NOT EDIT

#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <cake/base_node.hpp>
#include <cake/context.hpp>
#include <cake/publisher.hpp>
#include <cake/subscriber.hpp>
#include <cake/to_string.hpp>
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


template <
    typename ContextType,
    auto init_func,
    auto extend_options = [](rclcpp::NodeOptions options) { return options; }>
class ForEachParamBase : public cake::BaseNode<"for_each_param", extend_options> {
  public:
    explicit ForEachParamBase(const rclcpp::NodeOptions &options) : cake::BaseNode<"for_each_param", extend_options>(options) {
        static_assert(
            std::is_base_of_v<ForEachParamContext<ContextType>, ContextType>, "ContextType must be a child of ForEachParamContext"
        );

        // init context
        auto ctx = std::make_shared<ContextType>();
        ctx->node = this->node_;

        // init parameters (must be before publishers/subscribers for QoS param refs)
        ctx->param_listener = std::make_shared<ParamListener>(ctx->node);
        ctx->params = ctx->param_listener->get_params();

        // init publishers
        ctx->publishers.status = cake::create_publisher<std_msgs::msg::String>(ctx, "/robot/" + cake::to_string(ctx->params.robot_id) + "/status", rclcpp::QoS(10).reliable());
        // init subscribers
        for (const auto& key : ctx->params.managed_nodes) {
            ctx->subscribers.node_states[key] = cake::create_subscriber<std_msgs::msg::String>(ctx, "/" + key + "/state", rclcpp::QoS(10).reliable());
        }
        // init service clients
        for (const auto& key : ctx->params.managed_nodes) {
            ctx->service_clients.change_state_clients[key] = ctx->node->template create_client<lifecycle_msgs::srv::ChangeState>("/" + key + "/change_state");
        }
        init_func(ctx);
    }
};

} // namespace test_package::for_each_param
