// auto-generated DO NOT EDIT

#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/srv/add_two_ints.hpp>
#include <cake/base_node.hpp>
#include <cake/context.hpp>
#include <cake/service.hpp>

namespace test_package::service_node {

template <typename ContextType> struct ServiceNodePublishers {};

template <typename ContextType> struct ServiceNodeSubscribers {};

template <typename ContextType> struct ServiceNodeServices {
    std::shared_ptr<cake::Service<example_interfaces::srv::AddTwoInts, ContextType>> add_two_ints;
    std::shared_ptr<cake::Service<example_interfaces::srv::AddTwoInts, ContextType>> math_multiply;
};

template <typename DerivedContextType> struct ServiceNodeContext : cake::Context {
    ServiceNodePublishers<DerivedContextType> publishers;
    ServiceNodeSubscribers<DerivedContextType> subscribers;
    ServiceNodeServices<DerivedContextType> services;
};


template <
    typename ContextType,
    auto init_func,
    auto extend_options = [](rclcpp::NodeOptions options) { return options; }>
class ServiceNodeBase : public cake::BaseNode<"service_node", extend_options> {
  public:
    explicit ServiceNodeBase(const rclcpp::NodeOptions &options) : cake::BaseNode<"service_node", extend_options>(options) {
        static_assert(
            std::is_base_of_v<ServiceNodeContext<ContextType>, ContextType>, "ContextType must be a child of ServiceNodeContext"
        );

        // init context
        auto ctx = std::make_shared<ContextType>();
        ctx->node = this->node_;
        // init services
        ctx->services.add_two_ints = cake::create_service<example_interfaces::srv::AddTwoInts>(ctx, "add_two_ints", rclcpp::ServicesQoS());
        ctx->services.math_multiply = cake::create_service<example_interfaces::srv::AddTwoInts>(ctx, "/math/multiply", 10);
        init_func(ctx);
    }
};

} // namespace test_package::service_node
