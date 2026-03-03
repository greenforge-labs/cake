// auto-generated DO NOT EDIT

#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <example_interfaces/srv/add_two_ints.hpp>
#include <cake/base_node.hpp>
#include <cake/session.hpp>
#include <cake/service.hpp>
#include <test_package/services_only_parameters.hpp>

namespace test_package::services_only {

template <typename SessionType> struct ServicesOnlyPublishers {};

template <typename SessionType> struct ServicesOnlySubscribers {};

template <typename SessionType> struct ServicesOnlyServices {
    std::shared_ptr<cake::Service<example_interfaces::srv::AddTwoInts, SessionType>> add_two_ints;
    std::shared_ptr<cake::Service<example_interfaces::srv::AddTwoInts, SessionType>> math_multiply;
};

template <typename SessionType> struct ServicesOnlyServiceClients {};

template <typename SessionType> struct ServicesOnlyActions {};

template <typename SessionType> struct ServicesOnlyActionClients {};

template <typename DerivedSessionType> struct ServicesOnlySession : cake::Session {
    using cake::Session::Session;
    ServicesOnlyPublishers<DerivedSessionType> publishers;
    ServicesOnlySubscribers<DerivedSessionType> subscribers;
    ServicesOnlyServices<DerivedSessionType> services;
    ServicesOnlyServiceClients<DerivedSessionType> service_clients;
    ServicesOnlyActions<DerivedSessionType> actions;
    ServicesOnlyActionClients<DerivedSessionType> action_clients;
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
class ServicesOnlyBase : public cake::BaseNode<"services_only", SessionType, extend_options> {
    static_assert(
        std::is_base_of_v<ServicesOnlySession<SessionType>, SessionType>, "SessionType must be a child of ServicesOnlySession"
    );

  public:
    explicit ServicesOnlyBase(const rclcpp::NodeOptions &options)
        : cake::BaseNode<"services_only", SessionType, extend_options>(options) {}

  protected:
    std::shared_ptr<SessionType> create_session(rclcpp_lifecycle::LifecycleNode& node) override {
        auto sn = std::make_shared<SessionType>(node);
        // init parameters (must be before publishers/subscribers for QoS param refs)
        sn->param_listener = std::make_shared<ParamListener>(sn->node.shared_from_this());
        sn->params = sn->param_listener->get_params();
        // init services
        sn->services.add_two_ints = cake::create_service<example_interfaces::srv::AddTwoInts>(sn, "add_two_ints");
        sn->services.math_multiply = cake::create_service<example_interfaces::srv::AddTwoInts>(sn, "/math/multiply");
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

} // namespace test_package::services_only
