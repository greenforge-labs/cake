// auto-generated DO NOT EDIT

#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <cake/base_node.hpp>
#include <cake/context.hpp>
#include <cake/publisher.hpp>
#include <cake/subscriber.hpp>
#include <cake/qos_helpers.hpp>
#include <test_package/qos_param_substitution_parameters.hpp>

namespace test_package::qos_param_substitution {

template <typename ContextType> struct QosParamSubstitutionPublishers {
    std::shared_ptr<cake::Publisher<std_msgs::msg::String, ContextType>> processed_data;
};

template <typename ContextType> struct QosParamSubstitutionSubscribers {
    std::shared_ptr<cake::Subscriber<sensor_msgs::msg::LaserScan, ContextType>> sensor_data;
};

template <typename ContextType> struct QosParamSubstitutionServices {};

template <typename ContextType> struct QosParamSubstitutionServiceClients {};

template <typename ContextType> struct QosParamSubstitutionActions {};

template <typename ContextType> struct QosParamSubstitutionActionClients {};

template <typename DerivedContextType> struct QosParamSubstitutionContext : cake::Context {
    QosParamSubstitutionPublishers<DerivedContextType> publishers;
    QosParamSubstitutionSubscribers<DerivedContextType> subscribers;
    QosParamSubstitutionServices<DerivedContextType> services;
    QosParamSubstitutionServiceClients<DerivedContextType> service_clients;
    QosParamSubstitutionActions<DerivedContextType> actions;
    QosParamSubstitutionActionClients<DerivedContextType> action_clients;
    std::shared_ptr<ParamListener> param_listener;
    Params params;
};


template <
    typename ContextType,
    auto init_func,
    auto extend_options = [](rclcpp::NodeOptions options) { return options; }>
class QosParamSubstitutionBase : public cake::BaseNode<"qos_param_substitution", extend_options> {
  public:
    explicit QosParamSubstitutionBase(const rclcpp::NodeOptions &options) : cake::BaseNode<"qos_param_substitution", extend_options>(options) {
        static_assert(
            std::is_base_of_v<QosParamSubstitutionContext<ContextType>, ContextType>, "ContextType must be a child of QosParamSubstitutionContext"
        );

        // init context
        auto ctx = std::make_shared<ContextType>();
        ctx->node = this->node_;

        // init parameters (must be before publishers/subscribers for QoS param refs)
        ctx->param_listener = std::make_shared<ParamListener>(ctx->node);
        ctx->params = ctx->param_listener->get_params();

        // init publishers
        ctx->publishers.processed_data = cake::create_publisher<std_msgs::msg::String>(ctx, "/processed_data", rclcpp::QoS(ctx->params.output_queue_depth).reliable());
        // init subscribers
        ctx->subscribers.sensor_data = cake::create_subscriber<sensor_msgs::msg::LaserScan>(ctx, "/sensor_data", rclcpp::QoS(ctx->params.sensor_queue_depth).reliability(cake::to_reliability(ctx->params.sensor_reliability)).durability(cake::to_durability(ctx->params.sensor_durability)).deadline(rclcpp::Duration::from_nanoseconds(ctx->params.sensor_deadline_ms * 1000000LL)).liveliness(cake::to_liveliness(ctx->params.sensor_liveliness)));
        init_func(ctx);
    }
};

} // namespace test_package::qos_param_substitution
