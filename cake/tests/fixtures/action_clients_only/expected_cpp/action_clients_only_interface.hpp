// auto-generated DO NOT EDIT

#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <example_interfaces/action/fibonacci.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <cake/base_node.hpp>
#include <cake/session.hpp>
#include <test_package/action_clients_only_parameters.hpp>

namespace test_package::action_clients_only {

template <typename SessionType> struct ActionClientsOnlyPublishers {};

template <typename SessionType> struct ActionClientsOnlySubscribers {};

template <typename SessionType> struct ActionClientsOnlyServices {};

template <typename SessionType> struct ActionClientsOnlyServiceClients {};

template <typename SessionType> struct ActionClientsOnlyActions {};

template <typename SessionType> struct ActionClientsOnlyActionClients {
    rclcpp_action::Client<example_interfaces::action::Fibonacci>::SharedPtr fibonacci;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigate_to_pose;
};

template <typename DerivedSessionType> struct ActionClientsOnlySession : cake::Session {
    using cake::Session::Session;
    ActionClientsOnlyPublishers<DerivedSessionType> publishers;
    ActionClientsOnlySubscribers<DerivedSessionType> subscribers;
    ActionClientsOnlyServices<DerivedSessionType> services;
    ActionClientsOnlyServiceClients<DerivedSessionType> service_clients;
    ActionClientsOnlyActions<DerivedSessionType> actions;
    ActionClientsOnlyActionClients<DerivedSessionType> action_clients;
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
class ActionClientsOnlyBase : public cake::BaseNode<"action_clients_only", SessionType, extend_options> {
    static_assert(
        std::is_base_of_v<ActionClientsOnlySession<SessionType>, SessionType>, "SessionType must be a child of ActionClientsOnlySession"
    );

  public:
    explicit ActionClientsOnlyBase(const rclcpp::NodeOptions &options)
        : cake::BaseNode<"action_clients_only", SessionType, extend_options>(options) {}

  protected:
    std::shared_ptr<SessionType> create_session(rclcpp_lifecycle::LifecycleNode& node) override {
        auto sn = std::make_shared<SessionType>(node);
        // init parameters (must be before publishers/subscribers for QoS param refs)
        sn->param_listener = std::make_shared<ParamListener>(sn->node.shared_from_this());
        sn->params = sn->param_listener->get_params();
        // init action clients
        sn->action_clients.fibonacci = rclcpp_action::create_client<example_interfaces::action::Fibonacci>(sn->node.shared_from_this(), "/fibonacci", this->client_callback_group());
        sn->action_clients.navigate_to_pose = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(sn->node.shared_from_this(), "navigate_to_pose", this->client_callback_group());
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

} // namespace test_package::action_clients_only
