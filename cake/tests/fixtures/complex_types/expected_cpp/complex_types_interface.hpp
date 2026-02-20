// auto-generated DO NOT EDIT

#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
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

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

template <
    typename ContextType,
    auto on_configure_func,
    auto on_activate_func = [](std::shared_ptr<ContextType>) { return CallbackReturn::SUCCESS; },
    auto on_deactivate_func = [](std::shared_ptr<ContextType>) { return CallbackReturn::SUCCESS; },
    auto on_cleanup_func = [](std::shared_ptr<ContextType>) { return CallbackReturn::SUCCESS; },
    auto on_shutdown_func = [](std::shared_ptr<ContextType>) {},
    auto extend_options = [](rclcpp::NodeOptions options) { return options; }>
class ComplexTypesBase : public cake::BaseNode<"complex_types", ContextType, extend_options> {
    static_assert(
        std::is_base_of_v<ComplexTypesContext<ContextType>, ContextType>, "ContextType must be a child of ComplexTypesContext"
    );

  public:
    explicit ComplexTypesBase(const rclcpp::NodeOptions &options)
        : cake::BaseNode<"complex_types", ContextType, extend_options>(options) {}

  protected:
    void create_entities(std::shared_ptr<ContextType> ctx) override {
        // init parameters (must be before publishers/subscribers for QoS param refs)
        ctx->param_listener = std::make_shared<ParamListener>(ctx->node);
        ctx->params = ctx->param_listener->get_params();

        // init publishers
        ctx->publishers.pose = cake::create_publisher<geometry_msgs::msg::PoseStamped>(ctx, "pose", rclcpp::QoS(10).reliable());
        ctx->publishers.path = cake::create_publisher<nav_msgs::msg::Path>(ctx, "path", rclcpp::QoS(5).reliable());
        // init subscribers
        ctx->subscribers.joint_states = cake::create_subscriber<sensor_msgs::msg::JointState>(ctx, "joint_states", rclcpp::QoS(10).best_effort());
        ctx->subscribers.point_cloud = cake::create_subscriber<sensor_msgs::msg::PointCloud2>(ctx, "point_cloud", rclcpp::QoS(1).best_effort());
    }

    void activate_entities(std::shared_ptr<ContextType> ctx) override {
        ctx->publishers.pose->activate();
        ctx->publishers.path->activate();
        for (auto &t : ctx->timers) { t->reset(); }
    }

    void deactivate_entities(std::shared_ptr<ContextType> ctx) override {
        for (auto &t : ctx->timers) { t->cancel(); }
        if (ctx->publishers.pose) { ctx->publishers.pose->deactivate(); }
        if (ctx->publishers.path) { ctx->publishers.path->deactivate(); }
    }

    CallbackReturn user_on_configure(std::shared_ptr<ContextType> ctx) override { return on_configure_func(ctx); }
    CallbackReturn user_on_activate(std::shared_ptr<ContextType> ctx) override { return on_activate_func(ctx); }
    CallbackReturn user_on_deactivate(std::shared_ptr<ContextType> ctx) override { return on_deactivate_func(ctx); }
    CallbackReturn user_on_cleanup(std::shared_ptr<ContextType> ctx) override { return on_cleanup_func(ctx); }
    void user_on_shutdown(std::shared_ptr<ContextType> ctx) override { on_shutdown_func(ctx); }
};

} // namespace test_package::complex_types
