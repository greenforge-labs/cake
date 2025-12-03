// auto-generated DO NOT EDIT

#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <cake/base_node.hpp>
#include <cake/context.hpp>
#include <cake/publisher.hpp>
#include <test_package/publishers_only_parameters.hpp>

namespace test_package::publishers_only {

template <typename ContextType> struct PublishersOnlyPublishers {
    std::shared_ptr<cake::Publisher<std_msgs::msg::String, ContextType>> status;
    std::shared_ptr<cake::Publisher<std_msgs::msg::Int32, ContextType>> counter;
};

template <typename ContextType> struct PublishersOnlySubscribers {};

template <typename ContextType> struct PublishersOnlyServices {};

template <typename ContextType> struct PublishersOnlyServiceClients {};

template <typename ContextType> struct PublishersOnlyActions {};

template <typename ContextType> struct PublishersOnlyActionClients {};

template <typename DerivedContextType> struct PublishersOnlyContext : cake::Context {
    PublishersOnlyPublishers<DerivedContextType> publishers;
    PublishersOnlySubscribers<DerivedContextType> subscribers;
    PublishersOnlyServices<DerivedContextType> services;
    PublishersOnlyServiceClients<DerivedContextType> service_clients;
    PublishersOnlyActions<DerivedContextType> actions;
    PublishersOnlyActionClients<DerivedContextType> action_clients;
    std::shared_ptr<ParamListener> param_listener;
    Params params;
};


template <
    typename ContextType,
    auto init_func,
    auto extend_options = [](rclcpp::NodeOptions options) { return options; }>
class PublishersOnlyBase : public cake::BaseNode<"publishers_only", extend_options> {
  public:
    explicit PublishersOnlyBase(const rclcpp::NodeOptions &options) : cake::BaseNode<"publishers_only", extend_options>(options) {
        static_assert(
            std::is_base_of_v<PublishersOnlyContext<ContextType>, ContextType>, "ContextType must be a child of PublishersOnlyContext"
        );

        // init context
        auto ctx = std::make_shared<ContextType>();
        ctx->node = this->node_;

        // init publishers
        ctx->publishers.status = cake::create_publisher<std_msgs::msg::String>(ctx, "status", 10);
        ctx->publishers.counter = cake::create_publisher<std_msgs::msg::Int32>(ctx, "counter", 5);
        // init parameters
        ctx->param_listener = std::make_shared<ParamListener>(ctx->node);
        ctx->params = ctx->param_listener->get_params();

        init_func(ctx);
    }
};

} // namespace test_package::publishers_only
