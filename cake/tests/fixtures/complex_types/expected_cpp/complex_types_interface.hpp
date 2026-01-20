// auto-generated DO NOT EDIT

#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <cake/base_node.hpp>
#include <cake/context.hpp>
#include <cake/publisher.hpp>
#include <cake/subscriber.hpp>
#include <test_package/complex_types_parameters.hpp>

namespace test_package::complex_types {

template <typename ContextType> struct ComplexTypesPublishers {
    std::shared_ptr<cake::Publisher<geometry_msgs::msg::PoseStamped, ContextType>> pose;
    std::shared_ptr<cake::Publisher<nav_msgs::msg::Path, ContextType>> path;
};

template <typename ContextType> struct ComplexTypesSubscribers {
    std::shared_ptr<cake::Subscriber<sensor_msgs::msg::JointState, ContextType>> joint_states;
    std::shared_ptr<cake::Subscriber<sensor_msgs::msg::PointCloud2, ContextType>> point_cloud;
};

template <typename ContextType> struct ComplexTypesServices {};

template <typename ContextType> struct ComplexTypesServiceClients {};

template <typename ContextType> struct ComplexTypesActions {};

template <typename ContextType> struct ComplexTypesActionClients {};

template <typename DerivedContextType> struct ComplexTypesContext : cake::Context {
    ComplexTypesPublishers<DerivedContextType> publishers;
    ComplexTypesSubscribers<DerivedContextType> subscribers;
    ComplexTypesServices<DerivedContextType> services;
    ComplexTypesServiceClients<DerivedContextType> service_clients;
    ComplexTypesActions<DerivedContextType> actions;
    ComplexTypesActionClients<DerivedContextType> action_clients;
    std::shared_ptr<ParamListener> param_listener;
    Params params;
};


template <
    typename ContextType,
    auto init_func,
    auto extend_options = [](rclcpp::NodeOptions options) { return options; }>
class ComplexTypesBase : public cake::BaseNode<"complex_types", extend_options> {
  public:
    explicit ComplexTypesBase(const rclcpp::NodeOptions &options) : cake::BaseNode<"complex_types", extend_options>(options) {
        static_assert(
            std::is_base_of_v<ComplexTypesContext<ContextType>, ContextType>, "ContextType must be a child of ComplexTypesContext"
        );

        // init context
        auto ctx = std::make_shared<ContextType>();
        ctx->node = this->node_;

        // init parameters (must be before publishers/subscribers for QoS param refs)
        ctx->param_listener = std::make_shared<ParamListener>(ctx->node);
        ctx->params = ctx->param_listener->get_params();

        // init publishers
        ctx->publishers.pose = cake::create_publisher<geometry_msgs::msg::PoseStamped>(ctx, "pose", rclcpp::QoS(10).reliable());
        ctx->publishers.path = cake::create_publisher<nav_msgs::msg::Path>(ctx, "path", rclcpp::QoS(5).reliable());
        // init subscribers
        ctx->subscribers.joint_states = cake::create_subscriber<sensor_msgs::msg::JointState>(ctx, "joint_states", rclcpp::QoS(10).best_effort());
        ctx->subscribers.point_cloud = cake::create_subscriber<sensor_msgs::msg::PointCloud2>(ctx, "point_cloud", rclcpp::QoS(1).best_effort());
        init_func(ctx);
    }
};

} // namespace test_package::complex_types
