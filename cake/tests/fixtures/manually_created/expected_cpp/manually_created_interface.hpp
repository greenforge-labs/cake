// auto-generated DO NOT EDIT

#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <cake/base_node.hpp>
#include <cake/session.hpp>
#include <cake/publisher.hpp>
#include <cake/subscriber.hpp>
#include <cake/default_qos_handlers.hpp>
#include <test_package/manually_created_parameters.hpp>

namespace test_package::manually_created {

template <typename SessionType> struct ManuallyCreatedPublishers {
    std::shared_ptr<cake::Publisher<std_msgs::msg::String, SessionType>> auto_topic;
};

template <typename SessionType> struct ManuallyCreatedSubscribers {
    std::shared_ptr<cake::Subscriber<std_msgs::msg::Bool, SessionType>> auto_sub;
};

template <typename SessionType> struct ManuallyCreatedServices {};

template <typename SessionType> struct ManuallyCreatedServiceClients {};

template <typename SessionType> struct ManuallyCreatedActions {};

template <typename SessionType> struct ManuallyCreatedActionClients {};

template <typename DerivedSessionType> struct ManuallyCreatedSession : cake::Session {
    using cake::Session::Session;
    ManuallyCreatedPublishers<DerivedSessionType> publishers;
    ManuallyCreatedSubscribers<DerivedSessionType> subscribers;
    ManuallyCreatedServices<DerivedSessionType> services;
    ManuallyCreatedServiceClients<DerivedSessionType> service_clients;
    ManuallyCreatedActions<DerivedSessionType> actions;
    ManuallyCreatedActionClients<DerivedSessionType> action_clients;
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
class ManuallyCreatedBase : public cake::BaseNode<"manually_created", SessionType, extend_options> {
    static_assert(
        std::is_base_of_v<ManuallyCreatedSession<SessionType>, SessionType>, "SessionType must be a child of ManuallyCreatedSession"
    );

  public:
    explicit ManuallyCreatedBase(const rclcpp::NodeOptions &options)
        : cake::BaseNode<"manually_created", SessionType, extend_options>(options) {}

  protected:
    std::shared_ptr<SessionType> create_session(rclcpp_lifecycle::LifecycleNode& node) override {
        auto sn = std::make_shared<SessionType>(node);
        // init parameters (must be before publishers/subscribers for QoS param refs)
        sn->param_listener = std::make_shared<ParamListener>(sn->node.shared_from_this());
        sn->params = sn->param_listener->get_params();

        // init publishers
        sn->publishers.auto_topic = cake::create_publisher<std_msgs::msg::String>(sn, "auto_topic", rclcpp::QoS(10).reliable());
        // init subscribers
        sn->subscribers.auto_sub = cake::create_subscriber<std_msgs::msg::Bool>(sn, "auto_sub", rclcpp::QoS(10).best_effort());
        cake::attach_default_qos_handlers(sn->subscribers.auto_sub);
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

} // namespace test_package::manually_created
