// auto-generated DO NOT EDIT

#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <cake/base_node.hpp>
#include <cake/context.hpp>
#include <cake/subscriber.hpp>
#include <test_package/subscribers_only_parameters.hpp>

namespace test_package::subscribers_only {

template <typename ContextType> struct SubscribersOnlyPublishers {};

template <typename ContextType> struct SubscribersOnlySubscribers {
    std::shared_ptr<cake::Subscriber<sensor_msgs::msg::LaserScan, ContextType>> sensor_data;
    std::shared_ptr<cake::Subscriber<sensor_msgs::msg::Image, ContextType>> camera_image;
};

template <typename ContextType> struct SubscribersOnlyServices {};

template <typename ContextType> struct SubscribersOnlyServiceClients {};

template <typename ContextType> struct SubscribersOnlyActions {};

template <typename ContextType> struct SubscribersOnlyActionClients {};

template <typename DerivedContextType> struct SubscribersOnlyContext : cake::Context {
    SubscribersOnlyPublishers<DerivedContextType> publishers;
    SubscribersOnlySubscribers<DerivedContextType> subscribers;
    SubscribersOnlyServices<DerivedContextType> services;
    SubscribersOnlyServiceClients<DerivedContextType> service_clients;
    SubscribersOnlyActions<DerivedContextType> actions;
    SubscribersOnlyActionClients<DerivedContextType> action_clients;
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
class SubscribersOnlyBase : public cake::BaseNode<"subscribers_only", ContextType, extend_options> {
    static_assert(
        std::is_base_of_v<SubscribersOnlyContext<ContextType>, ContextType>, "ContextType must be a child of SubscribersOnlyContext"
    );

  public:
    explicit SubscribersOnlyBase(const rclcpp::NodeOptions &options)
        : cake::BaseNode<"subscribers_only", ContextType, extend_options>(options) {}

  protected:
    void create_entities(std::shared_ptr<ContextType> ctx) override {
        // init parameters (must be before publishers/subscribers for QoS param refs)
        ctx->param_listener = std::make_shared<ParamListener>(ctx->node);
        ctx->params = ctx->param_listener->get_params();
        // init subscribers
        ctx->subscribers.sensor_data = cake::create_subscriber<sensor_msgs::msg::LaserScan>(ctx, "sensor_data", rclcpp::QoS(10).best_effort());
        ctx->subscribers.camera_image = cake::create_subscriber<sensor_msgs::msg::Image>(ctx, "camera_image", rclcpp::QoS(1).best_effort());
    }

    void activate_entities(std::shared_ptr<ContextType> ctx) override {
        for (auto &t : ctx->timers) { t->reset(); }
    }

    void deactivate_entities(std::shared_ptr<ContextType> ctx) override {
        for (auto &t : ctx->timers) { t->cancel(); }
    }

    CallbackReturn user_on_configure(std::shared_ptr<ContextType> ctx) override { return on_configure_func(ctx); }
    CallbackReturn user_on_activate(std::shared_ptr<ContextType> ctx) override { return on_activate_func(ctx); }
    CallbackReturn user_on_deactivate(std::shared_ptr<ContextType> ctx) override { return on_deactivate_func(ctx); }
    CallbackReturn user_on_cleanup(std::shared_ptr<ContextType> ctx) override { return on_cleanup_func(ctx); }
    void user_on_shutdown(std::shared_ptr<ContextType> ctx) override { on_shutdown_func(ctx); }
};

} // namespace test_package::subscribers_only
