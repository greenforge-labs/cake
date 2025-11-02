// auto-generated DO NOT EDIT

#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/srv/add_two_ints.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <cake/base_node.hpp>
#include <cake/context.hpp>
#include <cake/service.hpp>

namespace test_package::default_qos_service_node {

template <typename ContextType> struct DefaultQosServiceNodePublishers {};

template <typename ContextType> struct DefaultQosServiceNodeSubscribers {};

template <typename ContextType> struct DefaultQosServiceNodeServices {
    std::shared_ptr<cake::Service<std_srvs::srv::Trigger, ContextType>> trigger_service;
    std::shared_ptr<cake::Service<example_interfaces::srv::AddTwoInts, ContextType>> compute;
};

template <typename ContextType> struct DefaultQosServiceNodeServiceClients {};

template <typename ContextType> struct DefaultQosServiceNodeActions {};

template <typename DerivedContextType> struct DefaultQosServiceNodeContext : cake::Context {
    DefaultQosServiceNodePublishers<DerivedContextType> publishers;
    DefaultQosServiceNodeSubscribers<DerivedContextType> subscribers;
    DefaultQosServiceNodeServices<DerivedContextType> services;
    DefaultQosServiceNodeServiceClients<DerivedContextType> service_clients;
    DefaultQosServiceNodeActions<DerivedContextType> actions;
};


template <
    typename ContextType,
    auto init_func,
    auto extend_options = [](rclcpp::NodeOptions options) { return options; }>
class DefaultQosServiceNodeBase : public cake::BaseNode<"default_qos_service_node", extend_options> {
  public:
    explicit DefaultQosServiceNodeBase(const rclcpp::NodeOptions &options) : cake::BaseNode<"default_qos_service_node", extend_options>(options) {
        static_assert(
            std::is_base_of_v<DefaultQosServiceNodeContext<ContextType>, ContextType>, "ContextType must be a child of DefaultQosServiceNodeContext"
        );

        // init context
        auto ctx = std::make_shared<ContextType>();
        ctx->node = this->node_;
        // init services
        ctx->services.trigger_service = cake::create_service<std_srvs::srv::Trigger>(ctx, "/trigger_service");
        ctx->services.compute = cake::create_service<example_interfaces::srv::AddTwoInts>(ctx, "compute", 10);
        init_func(ctx);
    }
};

} // namespace test_package::default_qos_service_node
