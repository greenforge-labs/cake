// auto-generated DO NOT EDIT

#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/srv/add_two_ints.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <cake/base_node.hpp>
#include <cake/context.hpp>
#include <cake/service.hpp>
#include <test_package/services_default_qos_parameters.hpp>

namespace test_package::services_default_qos {

template <typename ContextType> struct ServicesDefaultQosPublishers {};

template <typename ContextType> struct ServicesDefaultQosSubscribers {};

template <typename ContextType> struct ServicesDefaultQosServices {
    std::shared_ptr<cake::Service<std_srvs::srv::Trigger, ContextType>> trigger_service;
    std::shared_ptr<cake::Service<example_interfaces::srv::AddTwoInts, ContextType>> compute;
};

template <typename ContextType> struct ServicesDefaultQosServiceClients {};

template <typename ContextType> struct ServicesDefaultQosActions {};

template <typename ContextType> struct ServicesDefaultQosActionClients {};

template <typename DerivedContextType> struct ServicesDefaultQosContext : cake::Context {
    ServicesDefaultQosPublishers<DerivedContextType> publishers;
    ServicesDefaultQosSubscribers<DerivedContextType> subscribers;
    ServicesDefaultQosServices<DerivedContextType> services;
    ServicesDefaultQosServiceClients<DerivedContextType> service_clients;
    ServicesDefaultQosActions<DerivedContextType> actions;
    ServicesDefaultQosActionClients<DerivedContextType> action_clients;
    std::shared_ptr<ParamListener> param_listener;
    Params params;
};


template <
    typename ContextType,
    auto init_func,
    auto extend_options = [](rclcpp::NodeOptions options) { return options; }>
class ServicesDefaultQosBase : public cake::BaseNode<"services_default_qos", extend_options> {
  public:
    explicit ServicesDefaultQosBase(const rclcpp::NodeOptions &options) : cake::BaseNode<"services_default_qos", extend_options>(options) {
        static_assert(
            std::is_base_of_v<ServicesDefaultQosContext<ContextType>, ContextType>, "ContextType must be a child of ServicesDefaultQosContext"
        );

        // init context
        auto ctx = std::make_shared<ContextType>();
        ctx->node = this->node_;

        // init parameters (must be before publishers/subscribers for QoS param refs)
        ctx->param_listener = std::make_shared<ParamListener>(ctx->node);
        ctx->params = ctx->param_listener->get_params();
        // init services
        ctx->services.trigger_service = cake::create_service<std_srvs::srv::Trigger>(ctx, "/trigger_service");
        ctx->services.compute = cake::create_service<example_interfaces::srv::AddTwoInts>(ctx, "compute");
        init_func(ctx);
    }
};

} // namespace test_package::services_default_qos
