// auto-generated DO NOT EDIT

#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <cake/base_node.hpp>
#include <cake/session.hpp>
#include <cake/publisher.hpp>
#include <cake/subscriber.hpp>
#include <cake/default_qos_handlers.hpp>
#include <cake/service.hpp>
#include <test_package/name_param_substitution_parameters.hpp>

namespace test_package::name_param_substitution {

template <typename SessionType> struct NameParamSubstitutionPublishers {
    std::shared_ptr<cake::Publisher<geometry_msgs::msg::Twist, SessionType>> cmd_vel;
};

template <typename SessionType> struct NameParamSubstitutionSubscribers {
    std::shared_ptr<cake::Subscriber<nav_msgs::msg::Odometry, SessionType>> odom;
};

template <typename SessionType> struct NameParamSubstitutionServices {
    std::shared_ptr<cake::Service<std_srvs::srv::Trigger, SessionType>> get_state;
};

template <typename SessionType> struct NameParamSubstitutionServiceClients {
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr external_service;
};

template <typename SessionType> struct NameParamSubstitutionActions {};

template <typename SessionType> struct NameParamSubstitutionActionClients {};

template <typename DerivedSessionType> struct NameParamSubstitutionSession : cake::Session {
    using cake::Session::Session;
    NameParamSubstitutionPublishers<DerivedSessionType> publishers;
    NameParamSubstitutionSubscribers<DerivedSessionType> subscribers;
    NameParamSubstitutionServices<DerivedSessionType> services;
    NameParamSubstitutionServiceClients<DerivedSessionType> service_clients;
    NameParamSubstitutionActions<DerivedSessionType> actions;
    NameParamSubstitutionActionClients<DerivedSessionType> action_clients;
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
class NameParamSubstitutionBase : public cake::BaseNode<"name_param_substitution", SessionType, extend_options> {
    static_assert(
        std::is_base_of_v<NameParamSubstitutionSession<SessionType>, SessionType>, "SessionType must be a child of NameParamSubstitutionSession"
    );

  public:
    explicit NameParamSubstitutionBase(const rclcpp::NodeOptions &options)
        : cake::BaseNode<"name_param_substitution", SessionType, extend_options>(options) {}

  protected:
    std::shared_ptr<SessionType> create_session(rclcpp_lifecycle::LifecycleNode& node) override {
        auto sn = std::make_shared<SessionType>(node);
        // init parameters (must be before publishers/subscribers for QoS param refs)
        sn->param_listener = std::make_shared<ParamListener>(sn->node.shared_from_this());
        sn->params = sn->param_listener->get_params();

        // init publishers
        sn->publishers.cmd_vel = cake::create_publisher<geometry_msgs::msg::Twist>(sn, "/robot/" + sn->params.robot_id + "/cmd_vel", rclcpp::QoS(10).reliable());
        // init subscribers
        sn->subscribers.odom = cake::create_subscriber<nav_msgs::msg::Odometry>(sn, "/" + sn->params.namespace + "/" + sn->params.robot_id + "/odom", rclcpp::QoS(5).best_effort());
        cake::attach_default_qos_handlers(sn->subscribers.odom);
        // init services
        sn->services.get_state = cake::create_service<std_srvs::srv::Trigger>(sn, "/robot/" + sn->params.robot_id + "/get_state");
        // init service clients
        sn->service_clients.external_service = sn->node.template create_client<std_srvs::srv::SetBool>("/" + sn->params.namespace + "/service", rclcpp::ServicesQoS(), this->client_callback_group());
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

} // namespace test_package::name_param_substitution
