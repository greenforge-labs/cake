// auto-generated DO NOT EDIT

#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <cake/base_node.hpp>
#include <cake/context.hpp>
#include <cake/publisher.hpp>
#include <test_package/with_parameters_parameters.hpp>

namespace test_package::with_parameters {

template <typename ContextType> struct WithParametersPublishers {
    std::shared_ptr<cake::Publisher<std_msgs::msg::String, ContextType>> status;
};

template <typename ContextType> struct WithParametersSubscribers {};

template <typename ContextType> struct WithParametersServices {};

template <typename ContextType> struct WithParametersServiceClients {};

template <typename ContextType> struct WithParametersActions {};

template <typename ContextType> struct WithParametersActionClients {};

template <typename DerivedContextType> struct WithParametersContext : cake::Context {
    WithParametersPublishers<DerivedContextType> publishers;
    WithParametersSubscribers<DerivedContextType> subscribers;
    WithParametersServices<DerivedContextType> services;
    WithParametersServiceClients<DerivedContextType> service_clients;
    WithParametersActions<DerivedContextType> actions;
    WithParametersActionClients<DerivedContextType> action_clients;
    std::shared_ptr<ParamListener> param_listener;
    Params params;
};


template <
    typename ContextType,
    auto init_func,
    auto extend_options = [](rclcpp::NodeOptions options) { return options; }>
class WithParametersBase : public cake::BaseNode<"with_parameters", extend_options> {
  public:
    explicit WithParametersBase(const rclcpp::NodeOptions &options) : cake::BaseNode<"with_parameters", extend_options>(options) {
        static_assert(
            std::is_base_of_v<WithParametersContext<ContextType>, ContextType>, "ContextType must be a child of WithParametersContext"
        );

        // init context
        auto ctx = std::make_shared<ContextType>();
        ctx->node = this->node_;

        // init parameters (must be before publishers/subscribers for QoS param refs)
        ctx->param_listener = std::make_shared<ParamListener>(ctx->node);
        ctx->params = ctx->param_listener->get_params();

        // init publishers
        ctx->publishers.status = cake::create_publisher<std_msgs::msg::String>(ctx, "/status", rclcpp::QoS(10).reliable());
        init_func(ctx);
    }
};

} // namespace test_package::with_parameters
