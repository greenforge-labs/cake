// auto-generated DO NOT EDIT

#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/action/fibonacci.hpp>
#include <cake/base_node.hpp>
#include <cake/context.hpp>
#include <cake/action_server.hpp>
#include <test_package/action_node_parameters.hpp>

namespace test_package::action_node {

template <typename ContextType> struct ActionNodePublishers {};

template <typename ContextType> struct ActionNodeSubscribers {};

template <typename ContextType> struct ActionNodeServices {};

template <typename ContextType> struct ActionNodeServiceClients {};

template <typename ContextType> struct ActionNodeActions {
    std::shared_ptr<cake::SingleGoalActionServer<example_interfaces::action::Fibonacci>> fibonacci;
    std::shared_ptr<cake::SingleGoalActionServer<example_interfaces::action::Fibonacci>> math_compute;
};

template <typename DerivedContextType> struct ActionNodeContext : cake::Context {
    ActionNodePublishers<DerivedContextType> publishers;
    ActionNodeSubscribers<DerivedContextType> subscribers;
    ActionNodeServices<DerivedContextType> services;
    ActionNodeServiceClients<DerivedContextType> service_clients;
    ActionNodeActions<DerivedContextType> actions;
    std::shared_ptr<ParamListener> param_listener;
    Params params;
};


template <
    typename ContextType,
    auto init_func,
    auto extend_options = [](rclcpp::NodeOptions options) { return options; }>
class ActionNodeBase : public cake::BaseNode<"action_node", extend_options> {
  public:
    explicit ActionNodeBase(const rclcpp::NodeOptions &options) : cake::BaseNode<"action_node", extend_options>(options) {
        static_assert(
            std::is_base_of_v<ActionNodeContext<ContextType>, ContextType>, "ContextType must be a child of ActionNodeContext"
        );

        // init context
        auto ctx = std::make_shared<ContextType>();
        ctx->node = this->node_;
        // init actions
        ctx->actions.fibonacci = cake::create_single_goal_action_server<example_interfaces::action::Fibonacci>(ctx, "fibonacci");
        ctx->actions.math_compute = cake::create_single_goal_action_server<example_interfaces::action::Fibonacci>(ctx, "/math/compute");
        // init parameters
        ctx->param_listener = std::make_shared<ParamListener>(ctx->node);
        ctx->params = ctx->param_listener->get_params();

        init_func(ctx);
    }
};

} // namespace test_package::action_node
