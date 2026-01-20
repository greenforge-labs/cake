// auto-generated DO NOT EDIT

#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/srv/add_two_ints.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <cake/base_node.hpp>
#include <cake/context.hpp>
#include <cake/publisher.hpp>
#include <cake/subscriber.hpp>
#include <cake/service.hpp>
#include <test_package/service_clients_mixed_parameters.hpp>

namespace test_package::service_clients_mixed {

template <typename ContextType> struct ServiceClientsMixedPublishers {
    std::shared_ptr<cake::Publisher<std_msgs::msg::String, ContextType>> status;
};

template <typename ContextType> struct ServiceClientsMixedSubscribers {
    std::shared_ptr<cake::Subscriber<std_msgs::msg::Bool, ContextType>> command;
};

template <typename ContextType> struct ServiceClientsMixedServices {
    std::shared_ptr<cake::Service<std_srvs::srv::Trigger, ContextType>> reset;
};

template <typename ContextType> struct ServiceClientsMixedServiceClients {
    rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr add_two_ints;
    rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr compute;
};

template <typename ContextType> struct ServiceClientsMixedActions {};

template <typename ContextType> struct ServiceClientsMixedActionClients {};

template <typename DerivedContextType> struct ServiceClientsMixedContext : cake::Context {
    ServiceClientsMixedPublishers<DerivedContextType> publishers;
    ServiceClientsMixedSubscribers<DerivedContextType> subscribers;
    ServiceClientsMixedServices<DerivedContextType> services;
    ServiceClientsMixedServiceClients<DerivedContextType> service_clients;
    ServiceClientsMixedActions<DerivedContextType> actions;
    ServiceClientsMixedActionClients<DerivedContextType> action_clients;
    std::shared_ptr<ParamListener> param_listener;
    Params params;
};


template <
    typename ContextType,
    auto init_func,
    auto extend_options = [](rclcpp::NodeOptions options) { return options; }>
class ServiceClientsMixedBase : public cake::BaseNode<"service_clients_mixed", extend_options> {
  public:
    explicit ServiceClientsMixedBase(const rclcpp::NodeOptions &options) : cake::BaseNode<"service_clients_mixed", extend_options>(options) {
        static_assert(
            std::is_base_of_v<ServiceClientsMixedContext<ContextType>, ContextType>, "ContextType must be a child of ServiceClientsMixedContext"
        );

        // init context
        auto ctx = std::make_shared<ContextType>();
        ctx->node = this->node_;

        // init publishers
        ctx->publishers.status = cake::create_publisher<std_msgs::msg::String>(ctx, "/status", rclcpp::QoS(10).reliable());
        // init subscribers
        ctx->subscribers.command = cake::create_subscriber<std_msgs::msg::Bool>(ctx, "/command", rclcpp::QoS(5).best_effort());
        // init services
        ctx->services.reset = cake::create_service<std_srvs::srv::Trigger>(ctx, "/reset");
        // init service clients
        ctx->service_clients.add_two_ints = ctx->node->template create_client<example_interfaces::srv::AddTwoInts>("/add_two_ints");
        ctx->service_clients.compute = ctx->node->template create_client<example_interfaces::srv::AddTwoInts>("compute");
        // init parameters
        ctx->param_listener = std::make_shared<ParamListener>(ctx->node);
        ctx->params = ctx->param_listener->get_params();

        init_func(ctx);
    }
};

} // namespace test_package::service_clients_mixed
