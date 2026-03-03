// auto-generated DO NOT EDIT

#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <example_interfaces/action/fibonacci.hpp>
#include <example_interfaces/srv/add_two_ints.hpp>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <cake/base_node.hpp>
#include <cake/session.hpp>
#include <cake/publisher.hpp>
#include <cake/subscriber.hpp>
#include <cake/default_qos_handlers.hpp>
#include <cake/service.hpp>
#include <cake/action_server.hpp>
#include <test_package/action_clients_mixed_parameters.hpp>

namespace test_package::action_clients_mixed {

template <typename SessionType> struct ActionClientsMixedPublishers {
    std::shared_ptr<cake::Publisher<std_msgs::msg::String, SessionType>> status;
};

template <typename SessionType> struct ActionClientsMixedSubscribers {
    std::shared_ptr<cake::Subscriber<std_msgs::msg::Bool, SessionType>> command;
};

template <typename SessionType> struct ActionClientsMixedServices {
    std::shared_ptr<cake::Service<std_srvs::srv::Trigger, SessionType>> reset;
};

template <typename SessionType> struct ActionClientsMixedServiceClients {
    rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr compute;
};

template <typename SessionType> struct ActionClientsMixedActions {
    std::shared_ptr<cake::SingleGoalActionServer<example_interfaces::action::Fibonacci>> fibonacci_server;
};

template <typename SessionType> struct ActionClientsMixedActionClients {
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigate;
    rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>::SharedPtr compute_path;
};

template <typename DerivedSessionType> struct ActionClientsMixedSession : cake::Session {
    using cake::Session::Session;
    ActionClientsMixedPublishers<DerivedSessionType> publishers;
    ActionClientsMixedSubscribers<DerivedSessionType> subscribers;
    ActionClientsMixedServices<DerivedSessionType> services;
    ActionClientsMixedServiceClients<DerivedSessionType> service_clients;
    ActionClientsMixedActions<DerivedSessionType> actions;
    ActionClientsMixedActionClients<DerivedSessionType> action_clients;
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
class ActionClientsMixedBase : public cake::BaseNode<"action_clients_mixed", SessionType, extend_options> {
    static_assert(
        std::is_base_of_v<ActionClientsMixedSession<SessionType>, SessionType>, "SessionType must be a child of ActionClientsMixedSession"
    );

  public:
    explicit ActionClientsMixedBase(const rclcpp::NodeOptions &options)
        : cake::BaseNode<"action_clients_mixed", SessionType, extend_options>(options) {}

  protected:
    std::shared_ptr<SessionType> create_session(rclcpp_lifecycle::LifecycleNode& node) override {
        auto sn = std::make_shared<SessionType>(node);
        // init parameters (must be before publishers/subscribers for QoS param refs)
        sn->param_listener = std::make_shared<ParamListener>(sn->node.shared_from_this());
        sn->params = sn->param_listener->get_params();

        // init publishers
        sn->publishers.status = cake::create_publisher<std_msgs::msg::String>(sn, "/status", rclcpp::QoS(10).reliable());
        // init subscribers
        sn->subscribers.command = cake::create_subscriber<std_msgs::msg::Bool>(sn, "/command", rclcpp::QoS(5).best_effort());
        cake::attach_default_qos_handlers(sn->subscribers.command);
        // init services
        sn->services.reset = cake::create_service<std_srvs::srv::Trigger>(sn, "/reset");
        // init service clients
        sn->service_clients.compute = sn->node.template create_client<example_interfaces::srv::AddTwoInts>("/compute", rclcpp::ServicesQoS(), this->client_callback_group());
        // init actions
        sn->actions.fibonacci_server = cake::create_single_goal_action_server<example_interfaces::action::Fibonacci>(sn, "fibonacci_server");
        // init action clients
        sn->action_clients.navigate = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(sn->node.shared_from_this(), "/navigate", this->client_callback_group());
        sn->action_clients.compute_path = rclcpp_action::create_client<nav2_msgs::action::ComputePathToPose>(sn->node.shared_from_this(), "compute_path", this->client_callback_group());
        return sn;
    }

    void activate_entities(std::shared_ptr<SessionType> sn) override {
        for (auto &t : sn->timers) { t->reset(); }
    }

    void deactivate_entities(std::shared_ptr<SessionType> sn) override {
        for (auto &t : sn->timers) { t->cancel(); }
        if (sn->actions.fibonacci_server) { sn->actions.fibonacci_server->deactivate(); }
    }

    CallbackReturn user_on_configure(std::shared_ptr<SessionType> sn) override { return on_configure_func(sn); }
    CallbackReturn user_on_activate(std::shared_ptr<SessionType> sn) override { return on_activate_func(sn); }
    CallbackReturn user_on_deactivate(std::shared_ptr<SessionType> sn) override { return on_deactivate_func(sn); }
    CallbackReturn user_on_cleanup(std::shared_ptr<SessionType> sn) override { return on_cleanup_func(sn); }
    void user_on_shutdown(std::shared_ptr<SessionType> sn) override { on_shutdown_func(sn); }
};

} // namespace test_package::action_clients_mixed
