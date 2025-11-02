// auto-generated DO NOT EDIT

#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/srv/add_two_ints.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <cake/base_node.hpp>
#include <cake/context.hpp>
#include <cake/subscriber.hpp>
#include <cake/service.hpp>

namespace test_package::full_node {

template <typename ContextType> struct FullNodePublishers {
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status;
};

template <typename ContextType> struct FullNodeSubscribers {
    std::shared_ptr<cake::Subscriber<std_msgs::msg::Bool, ContextType>> command;
};

template <typename ContextType> struct FullNodeServices {
    std::shared_ptr<cake::Service<std_srvs::srv::Trigger, ContextType>> reset;
};

template <typename ContextType> struct FullNodeServiceClients {
    rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr add_two_ints;
    rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr compute;
};

template <typename ContextType> struct FullNodeActions {};

template <typename DerivedContextType> struct FullNodeContext : cake::Context {
    FullNodePublishers<DerivedContextType> publishers;
    FullNodeSubscribers<DerivedContextType> subscribers;
    FullNodeServices<DerivedContextType> services;
    FullNodeServiceClients<DerivedContextType> service_clients;
    FullNodeActions<DerivedContextType> actions;
};


template <
    typename ContextType,
    auto init_func,
    auto extend_options = [](rclcpp::NodeOptions options) { return options; }>
class FullNodeBase : public cake::BaseNode<"full_node", extend_options> {
  public:
    explicit FullNodeBase(const rclcpp::NodeOptions &options) : cake::BaseNode<"full_node", extend_options>(options) {
        static_assert(
            std::is_base_of_v<FullNodeContext<ContextType>, ContextType>, "ContextType must be a child of FullNodeContext"
        );

        // init context
        auto ctx = std::make_shared<ContextType>();
        ctx->node = this->node_;

        // init publishers
        ctx->publishers.status = ctx->node->template create_publisher<std_msgs::msg::String>("/status", 10);
        // init subscribers
        ctx->subscribers.command = cake::create_subscriber<std_msgs::msg::Bool>(ctx, "/command", rclcpp::SensorDataQoS());
        // init services
        ctx->services.reset = cake::create_service<std_srvs::srv::Trigger>(ctx, "/reset");
        // init service clients
        ctx->service_clients.add_two_ints = ctx->node->template create_client<example_interfaces::srv::AddTwoInts>("/add_two_ints", rclcpp::ServicesQoS());
        ctx->service_clients.compute = ctx->node->template create_client<example_interfaces::srv::AddTwoInts>("compute");
        init_func(ctx);
    }
};

} // namespace test_package::full_node
