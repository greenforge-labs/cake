// auto-generated DO NOT EDIT

#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/srv/add_two_ints.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <cake/base_node.hpp>
#include <cake/context.hpp>
#include <test_package/client_node_parameters.hpp>

namespace test_package::client_node {

template <typename ContextType> struct ClientNodePublishers {};

template <typename ContextType> struct ClientNodeSubscribers {};

template <typename ContextType> struct ClientNodeServices {};

template <typename ContextType> struct ClientNodeServiceClients {
    rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr add_two_ints;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr trigger_service;
};

template <typename ContextType> struct ClientNodeActions {};

template <typename DerivedContextType> struct ClientNodeContext : cake::Context {
    ClientNodePublishers<DerivedContextType> publishers;
    ClientNodeSubscribers<DerivedContextType> subscribers;
    ClientNodeServices<DerivedContextType> services;
    ClientNodeServiceClients<DerivedContextType> service_clients;
    ClientNodeActions<DerivedContextType> actions;
    std::shared_ptr<ParamListener> param_listener;
    Params params;
};


template <
    typename ContextType,
    auto init_func,
    auto extend_options = [](rclcpp::NodeOptions options) { return options; }>
class ClientNodeBase : public cake::BaseNode<"client_node", extend_options> {
  public:
    explicit ClientNodeBase(const rclcpp::NodeOptions &options) : cake::BaseNode<"client_node", extend_options>(options) {
        static_assert(
            std::is_base_of_v<ClientNodeContext<ContextType>, ContextType>, "ContextType must be a child of ClientNodeContext"
        );

        // init context
        auto ctx = std::make_shared<ContextType>();
        ctx->node = this->node_;
        // init service clients
        ctx->service_clients.add_two_ints = ctx->node->template create_client<example_interfaces::srv::AddTwoInts>("/add_two_ints");
        ctx->service_clients.trigger_service = ctx->node->template create_client<std_srvs::srv::Trigger>("trigger_service", 10);
        // init parameters
        ctx->param_listener = std::make_shared<ParamListener>(ctx->node);
        ctx->params = ctx->param_listener->get_params();

        init_func(ctx);
    }
};

} // namespace test_package::client_node
