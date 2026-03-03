// auto-generated DO NOT EDIT

#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <example_interfaces/action/fibonacci.hpp>
#include <cake/base_node.hpp>
#include <cake/session.hpp>
#include <cake/action_server.hpp>
#include <test_package/action_servers_only_parameters.hpp>

namespace test_package::action_servers_only {

template <typename SessionType> struct ActionServersOnlyPublishers {};

template <typename SessionType> struct ActionServersOnlySubscribers {};

template <typename SessionType> struct ActionServersOnlyServices {};

template <typename SessionType> struct ActionServersOnlyServiceClients {};

template <typename SessionType> struct ActionServersOnlyActions {
    std::shared_ptr<cake::SingleGoalActionServer<example_interfaces::action::Fibonacci>> fibonacci;
    std::shared_ptr<cake::SingleGoalActionServer<example_interfaces::action::Fibonacci>> math_compute;
};

template <typename SessionType> struct ActionServersOnlyActionClients {};

template <typename DerivedSessionType> struct ActionServersOnlySession : cake::Session {
    using cake::Session::Session;
    ActionServersOnlyPublishers<DerivedSessionType> publishers;
    ActionServersOnlySubscribers<DerivedSessionType> subscribers;
    ActionServersOnlyServices<DerivedSessionType> services;
    ActionServersOnlyServiceClients<DerivedSessionType> service_clients;
    ActionServersOnlyActions<DerivedSessionType> actions;
    ActionServersOnlyActionClients<DerivedSessionType> action_clients;
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
class ActionServersOnlyBase : public cake::BaseNode<"action_servers_only", SessionType, extend_options> {
    static_assert(
        std::is_base_of_v<ActionServersOnlySession<SessionType>, SessionType>, "SessionType must be a child of ActionServersOnlySession"
    );

  public:
    explicit ActionServersOnlyBase(const rclcpp::NodeOptions &options)
        : cake::BaseNode<"action_servers_only", SessionType, extend_options>(options) {}

  protected:
    std::shared_ptr<SessionType> create_session(rclcpp_lifecycle::LifecycleNode& node) override {
        auto sn = std::make_shared<SessionType>(node);
        // init parameters (must be before publishers/subscribers for QoS param refs)
        sn->param_listener = std::make_shared<ParamListener>(sn->node.shared_from_this());
        sn->params = sn->param_listener->get_params();
        // init actions
        sn->actions.fibonacci = cake::create_single_goal_action_server<example_interfaces::action::Fibonacci>(sn, "fibonacci");
        sn->actions.math_compute = cake::create_single_goal_action_server<example_interfaces::action::Fibonacci>(sn, "/math/compute");
        return sn;
    }

    void activate_entities(std::shared_ptr<SessionType> sn) override {
        for (auto &t : sn->timers) { t->reset(); }
    }

    void deactivate_entities(std::shared_ptr<SessionType> sn) override {
        for (auto &t : sn->timers) { t->cancel(); }
        if (sn->actions.fibonacci) { sn->actions.fibonacci->deactivate(); }
        if (sn->actions.math_compute) { sn->actions.math_compute->deactivate(); }
    }

    CallbackReturn user_on_configure(std::shared_ptr<SessionType> sn) override { return on_configure_func(sn); }
    CallbackReturn user_on_activate(std::shared_ptr<SessionType> sn) override { return on_activate_func(sn); }
    CallbackReturn user_on_deactivate(std::shared_ptr<SessionType> sn) override { return on_deactivate_func(sn); }
    CallbackReturn user_on_cleanup(std::shared_ptr<SessionType> sn) override { return on_cleanup_func(sn); }
    void user_on_shutdown(std::shared_ptr<SessionType> sn) override { on_shutdown_func(sn); }
};

} // namespace test_package::action_servers_only
