// auto-generated DO NOT EDIT

#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <cake/base_node.hpp>
#include <cake/context.hpp>
#include <cake/subscriber.hpp>
#include <test_package/qos_test_node_parameters.hpp>

namespace test_package::qos_test_node {

template <typename ContextType> struct QosTestNodePublishers {
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr sensor_data_topic;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr system_defaults_topic;
};

template <typename ContextType> struct QosTestNodeSubscribers {
    std::shared_ptr<cake::Subscriber<std_msgs::msg::String, ContextType>> parameters_topic;
    std::shared_ptr<cake::Subscriber<std_msgs::msg::String, ContextType>> services_topic;
};

template <typename ContextType> struct QosTestNodeServices {};

template <typename ContextType> struct QosTestNodeServiceClients {};

template <typename ContextType> struct QosTestNodeActions {};

template <typename ContextType> struct QosTestNodeActionClients {};

template <typename DerivedContextType> struct QosTestNodeContext : cake::Context {
    QosTestNodePublishers<DerivedContextType> publishers;
    QosTestNodeSubscribers<DerivedContextType> subscribers;
    QosTestNodeServices<DerivedContextType> services;
    QosTestNodeServiceClients<DerivedContextType> service_clients;
    QosTestNodeActions<DerivedContextType> actions;
    QosTestNodeActionClients<DerivedContextType> action_clients;
    std::shared_ptr<ParamListener> param_listener;
    Params params;
};


template <
    typename ContextType,
    auto init_func,
    auto extend_options = [](rclcpp::NodeOptions options) { return options; }>
class QosTestNodeBase : public cake::BaseNode<"qos_test_node", extend_options> {
  public:
    explicit QosTestNodeBase(const rclcpp::NodeOptions &options) : cake::BaseNode<"qos_test_node", extend_options>(options) {
        static_assert(
            std::is_base_of_v<QosTestNodeContext<ContextType>, ContextType>, "ContextType must be a child of QosTestNodeContext"
        );

        // init context
        auto ctx = std::make_shared<ContextType>();
        ctx->node = this->node_;

        // init publishers
        ctx->publishers.sensor_data_topic = ctx->node->template create_publisher<std_msgs::msg::String>("sensor_data_topic", rclcpp::SensorDataQoS());
        ctx->publishers.system_defaults_topic = ctx->node->template create_publisher<std_msgs::msg::String>("system_defaults_topic", rclcpp::SystemDefaultsQoS());
        // init subscribers
        ctx->subscribers.parameters_topic = cake::create_subscriber<std_msgs::msg::String>(ctx, "parameters_topic", rclcpp::ParametersQoS());
        ctx->subscribers.services_topic = cake::create_subscriber<std_msgs::msg::String>(ctx, "services_topic", rclcpp::ServicesQoS());
        // init parameters
        ctx->param_listener = std::make_shared<ParamListener>(ctx->node);
        ctx->params = ctx->param_listener->get_params();

        init_func(ctx);
    }
};

} // namespace test_package::qos_test_node
