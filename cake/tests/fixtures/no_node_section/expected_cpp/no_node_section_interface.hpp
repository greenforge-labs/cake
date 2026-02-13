// auto-generated DO NOT EDIT

#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <cake/base_node.hpp>
#include <cake/context.hpp>
#include <cake/publisher.hpp>
#include <cake/subscriber.hpp>
#include <test_package/no_node_section_parameters.hpp>

namespace test_package::no_node_section {

template <typename ContextType> struct NoNodeSectionPublishers {
    std::shared_ptr<cake::Publisher<std_msgs::msg::String, ContextType>> status;
};

template <typename ContextType> struct NoNodeSectionSubscribers {
    std::shared_ptr<cake::Subscriber<std_msgs::msg::Bool, ContextType>> input;
};

template <typename ContextType> struct NoNodeSectionServices {};

template <typename ContextType> struct NoNodeSectionServiceClients {};

template <typename ContextType> struct NoNodeSectionActions {};

template <typename ContextType> struct NoNodeSectionActionClients {};

template <typename DerivedContextType> struct NoNodeSectionContext : cake::Context {
    NoNodeSectionPublishers<DerivedContextType> publishers;
    NoNodeSectionSubscribers<DerivedContextType> subscribers;
    NoNodeSectionServices<DerivedContextType> services;
    NoNodeSectionServiceClients<DerivedContextType> service_clients;
    NoNodeSectionActions<DerivedContextType> actions;
    NoNodeSectionActionClients<DerivedContextType> action_clients;
    std::shared_ptr<ParamListener> param_listener;
    Params params;
};


template <
    typename ContextType,
    auto init_func,
    auto extend_options = [](rclcpp::NodeOptions options) { return options; }>
class NoNodeSectionBase : public cake::BaseNode<"no_node_section", extend_options> {
  public:
    explicit NoNodeSectionBase(const rclcpp::NodeOptions &options) : cake::BaseNode<"no_node_section", extend_options>(options) {
        static_assert(
            std::is_base_of_v<NoNodeSectionContext<ContextType>, ContextType>, "ContextType must be a child of NoNodeSectionContext"
        );

        // init context
        auto ctx = std::make_shared<ContextType>();
        ctx->node = this->node_;

        // init parameters (must be before publishers/subscribers for QoS param refs)
        ctx->param_listener = std::make_shared<ParamListener>(ctx->node);
        ctx->params = ctx->param_listener->get_params();

        // init publishers
        ctx->publishers.status = cake::create_publisher<std_msgs::msg::String>(ctx, "/status", rclcpp::QoS(10).reliable());
        // init subscribers
        ctx->subscribers.input = cake::create_subscriber<std_msgs::msg::Bool>(ctx, "/input", rclcpp::QoS(5).best_effort());
        init_func(ctx);
    }
};

} // namespace test_package::no_node_section
