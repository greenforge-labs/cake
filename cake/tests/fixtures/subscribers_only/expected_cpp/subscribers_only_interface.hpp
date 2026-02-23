// auto-generated DO NOT EDIT

#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <cake/base_node.hpp>
#include <cake/session.hpp>
#include <cake/subscriber.hpp>
#include <test_package/subscribers_only_parameters.hpp>

namespace test_package::subscribers_only {

template <typename SessionType> struct SubscribersOnlyPublishers {};

template <typename SessionType> struct SubscribersOnlySubscribers {
    std::shared_ptr<cake::Subscriber<sensor_msgs::msg::LaserScan, SessionType>> sensor_data;
    std::shared_ptr<cake::Subscriber<sensor_msgs::msg::Image, SessionType>> camera_image;
};

template <typename SessionType> struct SubscribersOnlyServices {};

template <typename SessionType> struct SubscribersOnlyServiceClients {};

template <typename SessionType> struct SubscribersOnlyActions {};

template <typename SessionType> struct SubscribersOnlyActionClients {};

template <typename DerivedSessionType> struct SubscribersOnlySession : cake::Session {
    using cake::Session::Session;
    SubscribersOnlyPublishers<DerivedSessionType> publishers;
    SubscribersOnlySubscribers<DerivedSessionType> subscribers;
    SubscribersOnlyServices<DerivedSessionType> services;
    SubscribersOnlyServiceClients<DerivedSessionType> service_clients;
    SubscribersOnlyActions<DerivedSessionType> actions;
    SubscribersOnlyActionClients<DerivedSessionType> action_clients;
    std::shared_ptr<ParamListener> param_listener;
    Params params;
};

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

template <
    typename SessionType,
    auto on_configure_func,
    auto on_activate_func = [](std::shared_ptr<SessionType>) { return CallbackReturn::SUCCESS; },
    auto on_deactivate_func = [](std::shared_ptr<SessionType>) { return CallbackReturn::SUCCESS; },
    auto on_cleanup_func = [](std::shared_ptr<SessionType>) { return CallbackReturn::SUCCESS; },
    auto on_shutdown_func = [](std::shared_ptr<SessionType>) {},
    auto extend_options = [](rclcpp::NodeOptions options) { return options; }>
class SubscribersOnlyBase : public cake::BaseNode<"subscribers_only", SessionType, extend_options> {
    static_assert(
        std::is_base_of_v<SubscribersOnlySession<SessionType>, SessionType>, "SessionType must be a child of SubscribersOnlySession"
    );

  public:
    explicit SubscribersOnlyBase(const rclcpp::NodeOptions &options)
        : cake::BaseNode<"subscribers_only", SessionType, extend_options>(options) {}

  protected:
    std::shared_ptr<SessionType> create_session(rclcpp_lifecycle::LifecycleNode& node) override {
        auto sn = std::make_shared<SessionType>(node);
        // init parameters (must be before publishers/subscribers for QoS param refs)
        sn->param_listener = std::make_shared<ParamListener>(sn->node.shared_from_this());
        sn->params = sn->param_listener->get_params();
        // init subscribers
        sn->subscribers.sensor_data = cake::create_subscriber<sensor_msgs::msg::LaserScan>(sn, "sensor_data", rclcpp::QoS(10).best_effort());
        sn->subscribers.camera_image = cake::create_subscriber<sensor_msgs::msg::Image>(sn, "camera_image", rclcpp::QoS(1).best_effort());
        return sn;
    }

    void activate_entities(std::shared_ptr<SessionType> sn) override {
        for (auto &t : sn->timers) { t->reset(); }
    }

    void deactivate_entities(std::shared_ptr<SessionType> sn) override {
        for (auto &t : sn->timers) { t->cancel(); }
    }

    CallbackReturn user_on_configure(std::shared_ptr<SessionType> sn) override { return on_configure_func(sn); }
    CallbackReturn user_on_activate(std::shared_ptr<SessionType> sn) override { return on_activate_func(sn); }
    CallbackReturn user_on_deactivate(std::shared_ptr<SessionType> sn) override { return on_deactivate_func(sn); }
    CallbackReturn user_on_cleanup(std::shared_ptr<SessionType> sn) override { return on_cleanup_func(sn); }
    void user_on_shutdown(std::shared_ptr<SessionType> sn) override { on_shutdown_func(sn); }
};

} // namespace test_package::subscribers_only
