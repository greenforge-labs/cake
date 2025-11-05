// auto-generated DO NOT EDIT

#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/srv/add_two_ints.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <cake/base_node.hpp>
#include <cake/context.hpp>
#include <cake/subscriber.hpp>
#include <cake/service.hpp>
#include <test_package/services_with_pubsub_parameters.hpp>

namespace test_package::services_with_pubsub {

template <typename ContextType> struct ServicesWithPubsubPublishers {
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status;
};

template <typename ContextType> struct ServicesWithPubsubSubscribers {
    std::shared_ptr<cake::Subscriber<std_msgs::msg::String, ContextType>> command;
};

template <typename ContextType> struct ServicesWithPubsubServices {
    std::shared_ptr<cake::Service<std_srvs::srv::Trigger, ContextType>> reset;
    std::shared_ptr<cake::Service<example_interfaces::srv::AddTwoInts, ContextType>> compute;
};

template <typename ContextType> struct ServicesWithPubsubServiceClients {};

template <typename ContextType> struct ServicesWithPubsubActions {};

template <typename ContextType> struct ServicesWithPubsubActionClients {};

template <typename DerivedContextType> struct ServicesWithPubsubContext : cake::Context {
    ServicesWithPubsubPublishers<DerivedContextType> publishers;
    ServicesWithPubsubSubscribers<DerivedContextType> subscribers;
    ServicesWithPubsubServices<DerivedContextType> services;
    ServicesWithPubsubServiceClients<DerivedContextType> service_clients;
    ServicesWithPubsubActions<DerivedContextType> actions;
    ServicesWithPubsubActionClients<DerivedContextType> action_clients;
    std::shared_ptr<ParamListener> param_listener;
    Params params;
};


template <
    typename ContextType,
    auto init_func,
    auto extend_options = [](rclcpp::NodeOptions options) { return options; }>
class ServicesWithPubsubBase : public cake::BaseNode<"services_with_pubsub", extend_options> {
  public:
    explicit ServicesWithPubsubBase(const rclcpp::NodeOptions &options) : cake::BaseNode<"services_with_pubsub", extend_options>(options) {
        static_assert(
            std::is_base_of_v<ServicesWithPubsubContext<ContextType>, ContextType>, "ContextType must be a child of ServicesWithPubsubContext"
        );

        // init context
        auto ctx = std::make_shared<ContextType>();
        ctx->node = this->node_;

        // init publishers
        ctx->publishers.status = ctx->node->template create_publisher<std_msgs::msg::String>("/status", 10);
        // init subscribers
        ctx->subscribers.command = cake::create_subscriber<std_msgs::msg::String>(ctx, "/command", rclcpp::SensorDataQoS());
        // init services
        ctx->services.reset = cake::create_service<std_srvs::srv::Trigger>(ctx, "/reset", rclcpp::ServicesQoS());
        ctx->services.compute = cake::create_service<example_interfaces::srv::AddTwoInts>(ctx, "compute", rclcpp::QoS(5).reliable());
        // init parameters
        ctx->param_listener = std::make_shared<ParamListener>(ctx->node);
        ctx->params = ctx->param_listener->get_params();

        init_func(ctx);
    }
};

} // namespace test_package::services_with_pubsub
