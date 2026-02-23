// auto-generated DO NOT EDIT

#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <example_interfaces/action/fibonacci.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <cake/base_node.hpp>
#include <cake/session.hpp>
#include <cake/publisher.hpp>
#include <cake/subscriber.hpp>
#include <cake/service.hpp>
#include <cake/action_server.hpp>
#include <test_package/action_servers_mixed_parameters.hpp>

namespace test_package::action_servers_mixed {

template <typename SessionType> struct ActionServersMixedPublishers {
    std::shared_ptr<cake::Publisher<std_msgs::msg::String, SessionType>> status;
};

template <typename SessionType> struct ActionServersMixedSubscribers {
    std::shared_ptr<cake::Subscriber<std_msgs::msg::Bool, SessionType>> cmd;
};

template <typename SessionType> struct ActionServersMixedServices {
    std::shared_ptr<cake::Service<std_srvs::srv::Trigger, SessionType>> reset;
};

template <typename SessionType> struct ActionServersMixedServiceClients {};

template <typename SessionType> struct ActionServersMixedActions {
    std::shared_ptr<cake::SingleGoalActionServer<example_interfaces::action::Fibonacci>> navigate;
};

template <typename SessionType> struct ActionServersMixedActionClients {};

template <typename DerivedSessionType> struct ActionServersMixedSession : cake::Session {
    using cake::Session::Session;
    ActionServersMixedPublishers<DerivedSessionType> publishers;
    ActionServersMixedSubscribers<DerivedSessionType> subscribers;
    ActionServersMixedServices<DerivedSessionType> services;
    ActionServersMixedServiceClients<DerivedSessionType> service_clients;
    ActionServersMixedActions<DerivedSessionType> actions;
    ActionServersMixedActionClients<DerivedSessionType> action_clients;
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
class ActionServersMixedBase : public cake::BaseNode<"action_servers_mixed", SessionType, extend_options> {
    static_assert(
        std::is_base_of_v<ActionServersMixedSession<SessionType>, SessionType>, "SessionType must be a child of ActionServersMixedSession"
    );

  public:
    explicit ActionServersMixedBase(const rclcpp::NodeOptions &options)
        : cake::BaseNode<"action_servers_mixed", SessionType, extend_options>(options) {}

  protected:
    std::shared_ptr<SessionType> create_session(rclcpp_lifecycle::LifecycleNode& node) override {
        auto sn = std::make_shared<SessionType>(node);
        // init parameters (must be before publishers/subscribers for QoS param refs)
        sn->param_listener = std::make_shared<ParamListener>(sn->node.shared_from_this());
        sn->params = sn->param_listener->get_params();

        // init publishers
        sn->publishers.status = cake::create_publisher<std_msgs::msg::String>(sn, "/status", rclcpp::QoS(10).reliable());
        // init subscribers
        sn->subscribers.cmd = cake::create_subscriber<std_msgs::msg::Bool>(sn, "/cmd", rclcpp::QoS(10).best_effort());
        // init services
        sn->services.reset = cake::create_service<std_srvs::srv::Trigger>(sn, "/reset");
        // init actions
        sn->actions.navigate = cake::create_single_goal_action_server<example_interfaces::action::Fibonacci>(sn, "navigate");
        return sn;
    }

    void activate_entities(std::shared_ptr<SessionType> sn) override {
        for (auto &t : sn->timers) { t->reset(); }
    }

    void deactivate_entities(std::shared_ptr<SessionType> sn) override {
        for (auto &t : sn->timers) { t->cancel(); }
        if (sn->actions.navigate) { sn->actions.navigate->deactivate(); }
    }

    CallbackReturn user_on_configure(std::shared_ptr<SessionType> sn) override { return on_configure_func(sn); }
    CallbackReturn user_on_activate(std::shared_ptr<SessionType> sn) override { return on_activate_func(sn); }
    CallbackReturn user_on_deactivate(std::shared_ptr<SessionType> sn) override { return on_deactivate_func(sn); }
    CallbackReturn user_on_cleanup(std::shared_ptr<SessionType> sn) override { return on_cleanup_func(sn); }
    void user_on_shutdown(std::shared_ptr<SessionType> sn) override { on_shutdown_func(sn); }
};

} // namespace test_package::action_servers_mixed
