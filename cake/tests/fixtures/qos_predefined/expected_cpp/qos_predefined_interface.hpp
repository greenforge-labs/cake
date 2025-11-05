// auto-generated DO NOT EDIT

#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <cake/base_node.hpp>
#include <cake/context.hpp>
#include <cake/subscriber.hpp>
#include <test_package/qos_predefined_parameters.hpp>

namespace test_package::qos_predefined {

template <typename ContextType> struct QosPredefinedPublishers {
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr sensor_data_topic;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr system_defaults_topic;
};

template <typename ContextType> struct QosPredefinedSubscribers {
    std::shared_ptr<cake::Subscriber<std_msgs::msg::String, ContextType>> parameters_topic;
    std::shared_ptr<cake::Subscriber<std_msgs::msg::String, ContextType>> services_topic;
};

template <typename ContextType> struct QosPredefinedServices {};

template <typename ContextType> struct QosPredefinedServiceClients {};

template <typename ContextType> struct QosPredefinedActions {};

template <typename ContextType> struct QosPredefinedActionClients {};

template <typename DerivedContextType> struct QosPredefinedContext : cake::Context {
    QosPredefinedPublishers<DerivedContextType> publishers;
    QosPredefinedSubscribers<DerivedContextType> subscribers;
    QosPredefinedServices<DerivedContextType> services;
    QosPredefinedServiceClients<DerivedContextType> service_clients;
    QosPredefinedActions<DerivedContextType> actions;
    QosPredefinedActionClients<DerivedContextType> action_clients;
    std::shared_ptr<ParamListener> param_listener;
    Params params;
};


template <
    typename ContextType,
    auto init_func,
    auto extend_options = [](rclcpp::NodeOptions options) { return options; }>
class QosPredefinedBase : public cake::BaseNode<"qos_predefined", extend_options> {
  public:
    explicit QosPredefinedBase(const rclcpp::NodeOptions &options) : cake::BaseNode<"qos_predefined", extend_options>(options) {
        static_assert(
            std::is_base_of_v<QosPredefinedContext<ContextType>, ContextType>, "ContextType must be a child of QosPredefinedContext"
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

} // namespace test_package::qos_predefined
