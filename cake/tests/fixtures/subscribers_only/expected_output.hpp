#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laserscan.hpp>

#include <cake/base_node.hpp>
#include <cake/context.hpp>
#include <cake/subscriber.hpp>

namespace sub_node {

template <typename ContextType> struct SubNodePublishers {};

template <typename ContextType> struct SubNodeSubscribers {
    std::shared_ptr<cake::Subscriber<sensor_msgs::msg::LaserScan, ContextType>> sensor_data;
    std::shared_ptr<cake::Subscriber<sensor_msgs::msg::Image, ContextType>> camera_image;
};

template <typename DerivedContextType> struct SubNodeContext : cake::Context {
    SubNodePublishers<DerivedContextType> publishers;
    SubNodeSubscribers<DerivedContextType> subscribers;
};

template <
    typename ContextType,
    auto init_func,
    auto extend_options = [](rclcpp::NodeOptions options) { return options; }>
class SubNodeBase : public cake::BaseNode<"sub_node", extend_options> {
  public:
    explicit SubNodeBase(const rclcpp::NodeOptions &options) : cake::BaseNode<"sub_node", extend_options>(options) {
        static_assert(
            std::is_base_of_v<SubNodeContext<ContextType>, ContextType>, "ContextType must be a child of SubNodeContext"
        );

        // init context
        auto ctx = std::make_shared<ContextType>();
        ctx->node = this->node_;

        // init subscribers
        ctx->subscribers.sensor_data = cake::create_subscriber<sensor_msgs::msg::LaserScan>(ctx, "sensor_data", 10);
        ctx->subscribers.camera_image = cake::create_subscriber<sensor_msgs::msg::Image>(ctx, "camera_image", 1);
        // TODO init services and actions

        init_func(ctx);
    }
};

} // namespace sub_node
