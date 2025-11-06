// auto-generated DO NOT EDIT

#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/srv/add_two_ints.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <cake/base_node.hpp>
#include <cake/context.hpp>
#include <test_package/service_clients_only_parameters.hpp>

namespace test_package::service_clients_only {

template <typename ContextType> struct ServiceClientsOnlyPublishers {};

template <typename ContextType> struct ServiceClientsOnlySubscribers {};

template <typename ContextType> struct ServiceClientsOnlyServices {};

template <typename ContextType> struct ServiceClientsOnlyServiceClients {
    rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr add_two_ints;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr trigger_service;
};

template <typename ContextType> struct ServiceClientsOnlyActions {};

template <typename ContextType> struct ServiceClientsOnlyActionClients {};

template <typename DerivedContextType> struct ServiceClientsOnlyContext : cake::Context {
    ServiceClientsOnlyPublishers<DerivedContextType> publishers;
    ServiceClientsOnlySubscribers<DerivedContextType> subscribers;
    ServiceClientsOnlyServices<DerivedContextType> services;
    ServiceClientsOnlyServiceClients<DerivedContextType> service_clients;
    ServiceClientsOnlyActions<DerivedContextType> actions;
    ServiceClientsOnlyActionClients<DerivedContextType> action_clients;
    std::shared_ptr<ParamListener> param_listener;
    Params params;
};


template <
    typename ContextType,
    auto init_func,
    auto extend_options = [](rclcpp::NodeOptions options) { return options; }>
class ServiceClientsOnlyBase : public cake::BaseNode<"service_clients_only", extend_options> {
  public:
    explicit ServiceClientsOnlyBase(const rclcpp::NodeOptions &options) : cake::BaseNode<"service_clients_only", extend_options>(options) {
        static_assert(
            std::is_base_of_v<ServiceClientsOnlyContext<ContextType>, ContextType>, "ContextType must be a child of ServiceClientsOnlyContext"
        );

        // init context
        auto ctx = std::make_shared<ContextType>();
        ctx->node = this->node_;
        // init service clients
        ctx->service_clients.add_two_ints = ctx->node->template create_client<example_interfaces::srv::AddTwoInts>("/add_two_ints");
        ctx->service_clients.trigger_service = ctx->node->template create_client<std_srvs::srv::Trigger>("trigger_service", rclcpp::QoS(10));
        // init parameters
        ctx->param_listener = std::make_shared<ParamListener>(ctx->node);
        ctx->params = ctx->param_listener->get_params();

        init_func(ctx);
    }
};

} // namespace test_package::service_clients_only
