// auto-generated DO NOT EDIT

#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <cake/base_node.hpp>
#include <cake/context.hpp>

namespace test_package::empty_node {

template <typename ContextType> struct EmptyNodePublishers {};

template <typename ContextType> struct EmptyNodeSubscribers {};

template <typename ContextType> struct EmptyNodeServices {};

template <typename ContextType> struct EmptyNodeServiceClients {};

template <typename ContextType> struct EmptyNodeActionServers {};

template <typename DerivedContextType> struct EmptyNodeContext : cake::Context {
    EmptyNodePublishers<DerivedContextType> publishers;
    EmptyNodeSubscribers<DerivedContextType> subscribers;
    EmptyNodeServices<DerivedContextType> services;
    EmptyNodeServiceClients<DerivedContextType> service_clients;
    EmptyNodeActionServers<DerivedContextType> action_servers;
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
        init_func(ctx);
    }
};

} // namespace test_package::empty_node
