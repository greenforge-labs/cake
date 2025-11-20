// auto-generated DO NOT EDIT

#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <cake/base_node.hpp>
#include <cake/context.hpp>
#include <cake/subscriber.hpp>
#include <test_package/qos_custom_parameters.hpp>

namespace test_package::qos_custom {

template <typename ContextType> struct QosCustomPublishers {
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr reliable_topic;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr best_effort_topic;
};

template <typename ContextType> struct QosCustomSubscribers {
    std::shared_ptr<cake::Subscriber<std_msgs::msg::String, ContextType>> keep_all_topic;
    std::shared_ptr<cake::Subscriber<std_msgs::msg::String, ContextType>> profile_override_topic;
};

template <typename ContextType> struct QosCustomServices {};

template <typename ContextType> struct QosCustomServiceClients {};

template <typename ContextType> struct QosCustomActions {};

template <typename ContextType> struct QosCustomActionClients {};

template <typename DerivedContextType> struct QosCustomContext : cake::Context {
    QosCustomPublishers<DerivedContextType> publishers;
    QosCustomSubscribers<DerivedContextType> subscribers;
    QosCustomServices<DerivedContextType> services;
    QosCustomServiceClients<DerivedContextType> service_clients;
    QosCustomActions<DerivedContextType> actions;
    QosCustomActionClients<DerivedContextType> action_clients;
    std::shared_ptr<ParamListener> param_listener;
    Params params;
};


template <
    typename ContextType,
    auto init_func,
    auto extend_options = [](rclcpp::NodeOptions options) { return options; }>
class QosCustomBase : public cake::BaseNode<"qos_custom", extend_options> {
  public:
    explicit QosCustomBase(const rclcpp::NodeOptions &options) : cake::BaseNode<"qos_custom", extend_options>(options) {
        static_assert(
            std::is_base_of_v<QosCustomContext<ContextType>, ContextType>, "ContextType must be a child of QosCustomContext"
        );

        // init context
        auto ctx = std::make_shared<ContextType>();
        ctx->node = this->node_;

        // init publishers
        ctx->publishers.reliable_topic = ctx->node->template create_publisher<std_msgs::msg::String>("reliable_topic", rclcpp::QoS(10).reliable().durability_volatile());
        ctx->publishers.best_effort_topic = ctx->node->template create_publisher<std_msgs::msg::String>("best_effort_topic", rclcpp::QoS(5).best_effort().transient_local());
        // init subscribers
        ctx->subscribers.keep_all_topic = cake::create_subscriber<std_msgs::msg::String>(ctx, "keep_all_topic", rclcpp::QoS(10).reliable().keep_all());
        ctx->subscribers.profile_override_topic = cake::create_subscriber<std_msgs::msg::String>(ctx, "profile_override_topic", rclcpp::SensorDataQoS().reliable().keep_last(20));
        // init parameters
        ctx->param_listener = std::make_shared<ParamListener>(ctx->node);
        ctx->params = ctx->param_listener->get_params();

        init_func(ctx);
    }
};

} // namespace test_package::qos_custom
