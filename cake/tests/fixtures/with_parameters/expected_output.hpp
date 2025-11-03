// auto-generated DO NOT EDIT

#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <cake/base_node.hpp>
#include <cake/context.hpp>
#include <test_package/test_node_parameters.hpp>

namespace test_package::test_node {

template <typename ContextType> struct TestNodePublishers {
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status;
};

template <typename ContextType> struct TestNodeSubscribers {};

template <typename ContextType> struct TestNodeServices {};

template <typename ContextType> struct TestNodeServiceClients {};

template <typename ContextType> struct TestNodeActions {};

template <typename DerivedContextType> struct TestNodeContext : cake::Context {
    TestNodePublishers<DerivedContextType> publishers;
    TestNodeSubscribers<DerivedContextType> subscribers;
    TestNodeServices<DerivedContextType> services;
    TestNodeServiceClients<DerivedContextType> service_clients;
    TestNodeActions<DerivedContextType> actions;
    std::shared_ptr<ParamListener> param_listener;
    Params params;
};


template <
    typename ContextType,
    auto init_func,
    auto extend_options = [](rclcpp::NodeOptions options) { return options; }>
class TestNodeBase : public cake::BaseNode<"test_node", extend_options> {
  public:
    explicit TestNodeBase(const rclcpp::NodeOptions &options) : cake::BaseNode<"test_node", extend_options>(options) {
        static_assert(
            std::is_base_of_v<TestNodeContext<ContextType>, ContextType>, "ContextType must be a child of TestNodeContext"
        );

        // init context
        auto ctx = std::make_shared<ContextType>();
        ctx->node = this->node_;

        // init publishers
        ctx->publishers.status = ctx->node->template create_publisher<std_msgs::msg::String>("/status", 10);
        // init parameters
        ctx->param_listener = std::make_shared<ParamListener>(ctx->node);
        ctx->params = ctx->param_listener->get_params();

        init_func(ctx);
    }
};

} // namespace test_package::test_node
