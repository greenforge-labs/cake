// auto-generated DO NOT EDIT

#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
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

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

template <
    typename ContextType,
    auto on_configure_func,
    auto on_activate_func = [](std::shared_ptr<ContextType>) { return CallbackReturn::SUCCESS; },
    auto on_deactivate_func = [](std::shared_ptr<ContextType>) { return CallbackReturn::SUCCESS; },
    auto on_cleanup_func = [](std::shared_ptr<ContextType>) { return CallbackReturn::SUCCESS; },
    auto on_shutdown_func = [](std::shared_ptr<ContextType>) {},
    auto extend_options = [](rclcpp::NodeOptions options) { return options; }>
class QosParamSubstitutionBase : public cake::BaseNode<"qos_param_substitution", ContextType, extend_options> {
    static_assert(
        std::is_base_of_v<QosParamSubstitutionContext<ContextType>, ContextType>, "ContextType must be a child of QosParamSubstitutionContext"
    );

  public:
    explicit QosParamSubstitutionBase(const rclcpp::NodeOptions &options)
        : cake::BaseNode<"qos_param_substitution", ContextType, extend_options>(options) {}

  protected:
    void create_entities(std::shared_ptr<ContextType> ctx) override {
        // init parameters (must be before publishers/subscribers for QoS param refs)
        ctx->param_listener = std::make_shared<ParamListener>(ctx->node);
        ctx->params = ctx->param_listener->get_params();

        // init publishers
        ctx->publishers.processed_data = cake::create_publisher<std_msgs::msg::String>(ctx, "/processed_data", rclcpp::QoS(ctx->params.output_queue_depth).reliable());
        // init subscribers
        ctx->subscribers.sensor_data = cake::create_subscriber<sensor_msgs::msg::LaserScan>(ctx, "/sensor_data", rclcpp::QoS(ctx->params.sensor_queue_depth).reliability(cake::to_reliability(ctx->params.sensor_reliability)).durability(cake::to_durability(ctx->params.sensor_durability)).deadline(rclcpp::Duration::from_nanoseconds(ctx->params.sensor_deadline_ms * 1000000LL)).liveliness(cake::to_liveliness(ctx->params.sensor_liveliness)));
    }

    void activate_entities(std::shared_ptr<ContextType> ctx) override {
        ctx->publishers.processed_data->activate();
        for (auto &t : ctx->timers) { t->reset(); }
    }

    void deactivate_entities(std::shared_ptr<ContextType> ctx) override {
        for (auto &t : ctx->timers) { t->cancel(); }
        if (ctx->publishers.processed_data) { ctx->publishers.processed_data->deactivate(); }
    }

    CallbackReturn user_on_configure(std::shared_ptr<ContextType> ctx) override { return on_configure_func(ctx); }
    CallbackReturn user_on_activate(std::shared_ptr<ContextType> ctx) override { return on_activate_func(ctx); }
    CallbackReturn user_on_deactivate(std::shared_ptr<ContextType> ctx) override { return on_deactivate_func(ctx); }
    CallbackReturn user_on_cleanup(std::shared_ptr<ContextType> ctx) override { return on_cleanup_func(ctx); }
    void user_on_shutdown(std::shared_ptr<ContextType> ctx) override { on_shutdown_func(ctx); }
};

} // namespace test_package::qos_param_substitution
