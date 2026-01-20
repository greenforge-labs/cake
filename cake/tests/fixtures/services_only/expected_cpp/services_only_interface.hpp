// auto-generated DO NOT EDIT

#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/srv/add_two_ints.hpp>
#include <cake/base_node.hpp>
#include <cake/context.hpp>
#include <cake/service.hpp>
#include <test_package/services_only_parameters.hpp>

namespace test_package::services_only {

template <typename ContextType> struct ServicesOnlyPublishers {};

template <typename ContextType> struct ServicesOnlySubscribers {};

template <typename ContextType> struct ServicesOnlyServices {
    std::shared_ptr<cake::Service<example_interfaces::srv::AddTwoInts, ContextType>> add_two_ints;
    std::shared_ptr<cake::Service<example_interfaces::srv::AddTwoInts, ContextType>> math_multiply;
};

template <typename ContextType> struct ServicesOnlyServiceClients {};

template <typename ContextType> struct ServicesOnlyActions {};

template <typename ContextType> struct ServicesOnlyActionClients {};

template <typename DerivedContextType> struct ServicesOnlyContext : cake::Context {
    ServicesOnlyPublishers<DerivedContextType> publishers;
    ServicesOnlySubscribers<DerivedContextType> subscribers;
    ServicesOnlyServices<DerivedContextType> services;
    ServicesOnlyServiceClients<DerivedContextType> service_clients;
    ServicesOnlyActions<DerivedContextType> actions;
    ServicesOnlyActionClients<DerivedContextType> action_clients;
    std::shared_ptr<ParamListener> param_listener;
    Params params;
};


template <
    typename ContextType,
    auto init_func,
    auto extend_options = [](rclcpp::NodeOptions options) { return options; }>
class ServicesOnlyBase : public cake::BaseNode<"services_only", extend_options> {
  public:
    explicit ServicesOnlyBase(const rclcpp::NodeOptions &options) : cake::BaseNode<"services_only", extend_options>(options) {
        static_assert(
            std::is_base_of_v<ServicesOnlyContext<ContextType>, ContextType>, "ContextType must be a child of ServicesOnlyContext"
        );

        // init context
        auto ctx = std::make_shared<ContextType>();
        ctx->node = this->node_;

        // init parameters (must be before publishers/subscribers for QoS param refs)
        ctx->param_listener = std::make_shared<ParamListener>(ctx->node);
        ctx->params = ctx->param_listener->get_params();
        // init services
        ctx->services.add_two_ints = cake::create_service<example_interfaces::srv::AddTwoInts>(ctx, "add_two_ints");
        ctx->services.math_multiply = cake::create_service<example_interfaces::srv::AddTwoInts>(ctx, "/math/multiply");
        init_func(ctx);
    }
};

} // namespace test_package::services_only
