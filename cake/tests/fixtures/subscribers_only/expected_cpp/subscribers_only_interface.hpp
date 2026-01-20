// auto-generated DO NOT EDIT

#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
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


template <
    typename ContextType,
    auto init_func,
    auto extend_options = [](rclcpp::NodeOptions options) { return options; }>
class SubscribersOnlyBase : public cake::BaseNode<"subscribers_only", extend_options> {
  public:
    explicit SubscribersOnlyBase(const rclcpp::NodeOptions &options) : cake::BaseNode<"subscribers_only", extend_options>(options) {
        static_assert(
            std::is_base_of_v<SubscribersOnlyContext<ContextType>, ContextType>, "ContextType must be a child of SubscribersOnlyContext"
        );

        // init context
        auto ctx = std::make_shared<ContextType>();
        ctx->node = this->node_;
        // init subscribers
        ctx->subscribers.sensor_data = cake::create_subscriber<sensor_msgs::msg::LaserScan>(ctx, "sensor_data", rclcpp::QoS(10).best_effort());
        ctx->subscribers.camera_image = cake::create_subscriber<sensor_msgs::msg::Image>(ctx, "camera_image", rclcpp::QoS(1).best_effort());
        // init parameters
        ctx->param_listener = std::make_shared<ParamListener>(ctx->node);
        ctx->params = ctx->param_listener->get_params();

        init_func(ctx);
    }
};

} // namespace test_package::subscribers_only
