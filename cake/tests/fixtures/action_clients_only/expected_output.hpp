// auto-generated DO NOT EDIT

#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/action/fibonacci.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <cake/base_node.hpp>
#include <cake/context.hpp>
#include <test_package/action_client_node_parameters.hpp>

namespace test_package::action_client_node {

template <typename ContextType> struct ActionClientNodePublishers {};

template <typename ContextType> struct ActionClientNodeSubscribers {};

template <typename ContextType> struct ActionClientNodeServices {};

template <typename ContextType> struct ActionClientNodeServiceClients {};

template <typename ContextType> struct ActionClientNodeActions {};

template <typename ContextType> struct ActionClientNodeActionClients {
    rclcpp_action::Client<example_interfaces::action::Fibonacci>::SharedPtr fibonacci;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigate_to_pose;
};

template <typename DerivedContextType> struct ActionClientNodeContext : cake::Context {
    ActionClientNodePublishers<DerivedContextType> publishers;
    ActionClientNodeSubscribers<DerivedContextType> subscribers;
    ActionClientNodeServices<DerivedContextType> services;
    ActionClientNodeServiceClients<DerivedContextType> service_clients;
    ActionClientNodeActions<DerivedContextType> actions;
    ActionClientNodeActionClients<DerivedContextType> action_clients;
    std::shared_ptr<ParamListener> param_listener;
    Params params;
};


template <
    typename ContextType,
    auto init_func,
    auto extend_options = [](rclcpp::NodeOptions options) { return options; }>
class ActionClientNodeBase : public cake::BaseNode<"action_client_node", extend_options> {
  public:
    explicit ActionClientNodeBase(const rclcpp::NodeOptions &options) : cake::BaseNode<"action_client_node", extend_options>(options) {
        static_assert(
            std::is_base_of_v<ActionClientNodeContext<ContextType>, ContextType>, "ContextType must be a child of ActionClientNodeContext"
        );

        // init context
        auto ctx = std::make_shared<ContextType>();
        ctx->node = this->node_;
        // init action clients
        ctx->action_clients.fibonacci = rclcpp_action::create_client<example_interfaces::action::Fibonacci>(ctx->node, "/fibonacci");
        ctx->action_clients.navigate_to_pose = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(ctx->node, "navigate_to_pose");
        // init parameters
        ctx->param_listener = std::make_shared<ParamListener>(ctx->node);
        ctx->params = ctx->param_listener->get_params();

        init_func(ctx);
    }
};

} // namespace test_package::action_client_node
