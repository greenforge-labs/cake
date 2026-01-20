// auto-generated DO NOT EDIT

#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <cake/base_node.hpp>
#include <cake/context.hpp>
#include <cake/publisher.hpp>
#include <cake/subscriber.hpp>
#include <test_package/qos_backward_compat_parameters.hpp>

namespace test_package::qos_backward_compat {

template <typename ContextType> struct QosBackwardCompatPublishers {
    std::shared_ptr<cake::Publisher<std_msgs::msg::String, ContextType>> int_qos_pub;
    std::shared_ptr<cake::Publisher<std_msgs::msg::String, ContextType>> default_qos_pub;
};

template <typename ContextType> struct QosBackwardCompatSubscribers {
    std::shared_ptr<cake::Subscriber<std_msgs::msg::String, ContextType>> int_qos_sub;
    std::shared_ptr<cake::Subscriber<std_msgs::msg::String, ContextType>> default_qos_sub;
};

template <typename ContextType> struct QosBackwardCompatServices {};

template <typename ContextType> struct QosBackwardCompatServiceClients {};

template <typename ContextType> struct QosBackwardCompatActions {};

template <typename ContextType> struct QosBackwardCompatActionClients {};

template <typename DerivedContextType> struct QosBackwardCompatContext : cake::Context {
    QosBackwardCompatPublishers<DerivedContextType> publishers;
    QosBackwardCompatSubscribers<DerivedContextType> subscribers;
    QosBackwardCompatServices<DerivedContextType> services;
    QosBackwardCompatServiceClients<DerivedContextType> service_clients;
    QosBackwardCompatActions<DerivedContextType> actions;
    QosBackwardCompatActionClients<DerivedContextType> action_clients;
    std::shared_ptr<ParamListener> param_listener;
    Params params;
};


template <
    typename ContextType,
    auto init_func,
    auto extend_options = [](rclcpp::NodeOptions options) { return options; }>
class QosBackwardCompatBase : public cake::BaseNode<"qos_backward_compat", extend_options> {
  public:
    explicit QosBackwardCompatBase(const rclcpp::NodeOptions &options) : cake::BaseNode<"qos_backward_compat", extend_options>(options) {
        static_assert(
            std::is_base_of_v<QosBackwardCompatContext<ContextType>, ContextType>, "ContextType must be a child of QosBackwardCompatContext"
        );

        // init context
        auto ctx = std::make_shared<ContextType>();
        ctx->node = this->node_;

        // init publishers
        ctx->publishers.int_qos_pub = cake::create_publisher<std_msgs::msg::String>(ctx, "int_qos_pub", rclcpp::QoS(10).best_effort());
        ctx->publishers.default_qos_pub = cake::create_publisher<std_msgs::msg::String>(ctx, "default_qos_pub", rclcpp::QoS(10).reliable());
        // init subscribers
        ctx->subscribers.int_qos_sub = cake::create_subscriber<std_msgs::msg::String>(ctx, "int_qos_sub", rclcpp::QoS(5).best_effort());
        ctx->subscribers.default_qos_sub = cake::create_subscriber<std_msgs::msg::String>(ctx, "default_qos_sub", rclcpp::QoS(10).reliable());
        // init parameters
        ctx->param_listener = std::make_shared<ParamListener>(ctx->node);
        ctx->params = ctx->param_listener->get_params();

        init_func(ctx);
    }
};

} // namespace test_package::qos_backward_compat
