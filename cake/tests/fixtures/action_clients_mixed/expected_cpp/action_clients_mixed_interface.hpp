// auto-generated DO NOT EDIT

#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/action/fibonacci.hpp>
#include <example_interfaces/srv/add_two_ints.hpp>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <cake/base_node.hpp>
#include <cake/context.hpp>
#include <cake/publisher.hpp>
#include <cake/subscriber.hpp>
#include <cake/service.hpp>
#include <cake/action_server.hpp>
#include <test_package/action_clients_mixed_parameters.hpp>

namespace test_package::action_clients_mixed {

template <typename ContextType> struct ActionClientsMixedPublishers {
    std::shared_ptr<cake::Publisher<std_msgs::msg::String, ContextType>> status;
};

template <typename ContextType> struct ActionClientsMixedSubscribers {
    std::shared_ptr<cake::Subscriber<std_msgs::msg::Bool, ContextType>> command;
};

template <typename ContextType> struct ActionClientsMixedServices {
    std::shared_ptr<cake::Service<std_srvs::srv::Trigger, ContextType>> reset;
};

template <typename ContextType> struct ActionClientsMixedServiceClients {
    rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr compute;
};

template <typename ContextType> struct ActionClientsMixedActions {
    std::shared_ptr<cake::SingleGoalActionServer<example_interfaces::action::Fibonacci>> fibonacci_server;
};

template <typename ContextType> struct ActionClientsMixedActionClients {
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigate;
    rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>::SharedPtr compute_path;
};

template <typename DerivedContextType> struct ActionClientsMixedContext : cake::Context {
    ActionClientsMixedPublishers<DerivedContextType> publishers;
    ActionClientsMixedSubscribers<DerivedContextType> subscribers;
    ActionClientsMixedServices<DerivedContextType> services;
    ActionClientsMixedServiceClients<DerivedContextType> service_clients;
    ActionClientsMixedActions<DerivedContextType> actions;
    ActionClientsMixedActionClients<DerivedContextType> action_clients;
    std::shared_ptr<ParamListener> param_listener;
    Params params;
};


template <
    typename ContextType,
    auto init_func,
    auto extend_options = [](rclcpp::NodeOptions options) { return options; }>
class ActionClientsMixedBase : public cake::BaseNode<"action_clients_mixed", extend_options> {
  public:
    explicit ActionClientsMixedBase(const rclcpp::NodeOptions &options) : cake::BaseNode<"action_clients_mixed", extend_options>(options) {
        static_assert(
            std::is_base_of_v<ActionClientsMixedContext<ContextType>, ContextType>, "ContextType must be a child of ActionClientsMixedContext"
        );

        // init context
        auto ctx = std::make_shared<ContextType>();
        ctx->node = this->node_;

        // init publishers
        ctx->publishers.status = cake::create_publisher<std_msgs::msg::String>(ctx, "/status", 10);
        // init subscribers
        ctx->subscribers.command = cake::create_subscriber<std_msgs::msg::Bool>(ctx, "/command", rclcpp::SensorDataQoS());
        // init services
        ctx->services.reset = cake::create_service<std_srvs::srv::Trigger>(ctx, "/reset");
        // init service clients
        ctx->service_clients.compute = ctx->node->template create_client<example_interfaces::srv::AddTwoInts>("/compute");
        // init actions
        ctx->actions.fibonacci_server = cake::create_single_goal_action_server<example_interfaces::action::Fibonacci>(ctx, "fibonacci_server");
        // init action clients
        ctx->action_clients.navigate = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(ctx->node, "/navigate");
        ctx->action_clients.compute_path = rclcpp_action::create_client<nav2_msgs::action::ComputePathToPose>(ctx->node, "compute_path");
        // init parameters
        ctx->param_listener = std::make_shared<ParamListener>(ctx->node);
        ctx->params = ctx->param_listener->get_params();

        init_func(ctx);
    }
};

} // namespace test_package::action_clients_mixed
