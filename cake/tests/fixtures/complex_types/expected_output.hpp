// auto-generated DO NOT EDIT

#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/posestamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/jointstate.hpp>
#include <sensor_msgs/msg/pointcloud2.hpp>
#include <cake/base_node.hpp>
#include <cake/context.hpp>
#include <cake/subscriber.hpp>

namespace test_package::complex_node {

template <typename ContextType> struct ComplexNodePublishers {
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path;
};

template <typename ContextType> struct ComplexNodeSubscribers {
    std::shared_ptr<cake::Subscriber<sensor_msgs::msg::JointState, ContextType>> joint_states;
    std::shared_ptr<cake::Subscriber<sensor_msgs::msg::PointCloud2, ContextType>> point_cloud;
};

template <typename DerivedContextType> struct ComplexNodeContext : cake::Context {
    ComplexNodePublishers<DerivedContextType> publishers;
    ComplexNodeSubscribers<DerivedContextType> subscribers;
};


template <
    typename ContextType,
    auto init_func,
    auto extend_options = [](rclcpp::NodeOptions options) { return options; }>
class ComplexNodeBase : public cake::BaseNode<"complex_node", extend_options> {
  public:
    explicit ComplexNodeBase(const rclcpp::NodeOptions &options) : cake::BaseNode<"complex_node", extend_options>(options) {
        static_assert(
            std::is_base_of_v<ComplexNodeContext<ContextType>, ContextType>, "ContextType must be a child of ComplexNodeContext"
        );

        // init context
        auto ctx = std::make_shared<ContextType>();
        ctx->node = this->node_;

        // init publishers
        ctx->publishers.pose = ctx->node->template create_publisher<geometry_msgs::msg::PoseStamped>("pose", 10);
        ctx->publishers.path = ctx->node->template create_publisher<nav_msgs::msg::Path>("path", 5);
        // init subscribers
        ctx->subscribers.joint_states = cake::create_subscriber<sensor_msgs::msg::JointState>(ctx, "joint_states", 10);
        ctx->subscribers.point_cloud = cake::create_subscriber<sensor_msgs::msg::PointCloud2>(ctx, "point_cloud", 1);
        // TODO init services and actions

        init_func(ctx);
    }
};

} // namespace test_package::complex_node
