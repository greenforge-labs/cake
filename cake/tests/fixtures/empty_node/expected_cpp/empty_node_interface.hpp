// auto-generated DO NOT EDIT

#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <cake/base_node.hpp>
#include <cake/context.hpp>
#include <test_package/empty_node_parameters.hpp>

namespace test_package::empty_node {

template <typename ContextType> struct EmptyNodePublishers {};

template <typename ContextType> struct EmptyNodeSubscribers {};

template <typename ContextType> struct EmptyNodeServices {};

template <typename ContextType> struct EmptyNodeServiceClients {};

template <typename ContextType> struct EmptyNodeActions {};

template <typename ContextType> struct EmptyNodeActionClients {};

template <typename DerivedContextType> struct EmptyNodeContext : cake::Context {
    EmptyNodePublishers<DerivedContextType> publishers;
    EmptyNodeSubscribers<DerivedContextType> subscribers;
    EmptyNodeServices<DerivedContextType> services;
    EmptyNodeServiceClients<DerivedContextType> service_clients;
    EmptyNodeActions<DerivedContextType> actions;
    EmptyNodeActionClients<DerivedContextType> action_clients;
    std::shared_ptr<ParamListener> param_listener;
    Params params;
};


template <
    typename ContextType,
    auto init_func,
    auto extend_options = [](rclcpp::NodeOptions options) { return options; }>
class EmptyNodeBase : public cake::BaseNode<"empty_node", extend_options> {
  public:
    explicit EmptyNodeBase(const rclcpp::NodeOptions &options) : cake::BaseNode<"empty_node", extend_options>(options) {
        static_assert(
            std::is_base_of_v<EmptyNodeContext<ContextType>, ContextType>, "ContextType must be a child of EmptyNodeContext"
        );

        // init context
        auto ctx = std::make_shared<ContextType>();
        ctx->node = this->node_;

        // init parameters (must be before publishers/subscribers for QoS param refs)
        ctx->param_listener = std::make_shared<ParamListener>(ctx->node);
        ctx->params = ctx->param_listener->get_params();
        init_func(ctx);
    }
};

} // namespace test_package::empty_node
