// auto-generated DO NOT EDIT

#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <example_interfaces/srv/add_two_ints.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <cake/base_node.hpp>
#include <cake/session.hpp>
#include <cake/service.hpp>
#include <test_package/services_default_qos_parameters.hpp>

namespace test_package::services_default_qos {

template <typename SessionType> struct ServicesDefaultQosPublishers {};

template <typename SessionType> struct ServicesDefaultQosSubscribers {};

template <typename SessionType> struct ServicesDefaultQosServices {
    std::shared_ptr<cake::Service<std_srvs::srv::Trigger, SessionType>> trigger_service;
    std::shared_ptr<cake::Service<example_interfaces::srv::AddTwoInts, SessionType>> compute;
};

template <typename SessionType> struct ServicesDefaultQosServiceClients {};

template <typename SessionType> struct ServicesDefaultQosActions {};

template <typename SessionType> struct ServicesDefaultQosActionClients {};

template <typename DerivedSessionType> struct ServicesDefaultQosSession : cake::Session {
    using cake::Session::Session;
    ServicesDefaultQosPublishers<DerivedSessionType> publishers;
    ServicesDefaultQosSubscribers<DerivedSessionType> subscribers;
    ServicesDefaultQosServices<DerivedSessionType> services;
    ServicesDefaultQosServiceClients<DerivedSessionType> service_clients;
    ServicesDefaultQosActions<DerivedSessionType> actions;
    ServicesDefaultQosActionClients<DerivedSessionType> action_clients;
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
class ServicesDefaultQosBase : public cake::BaseNode<"services_default_qos", SessionType, extend_options> {
    static_assert(
        std::is_base_of_v<ServicesDefaultQosSession<SessionType>, SessionType>, "SessionType must be a child of ServicesDefaultQosSession"
    );

  public:
    explicit ServicesDefaultQosBase(const rclcpp::NodeOptions &options)
        : cake::BaseNode<"services_default_qos", SessionType, extend_options>(options) {}

  protected:
    std::shared_ptr<SessionType> create_session(rclcpp_lifecycle::LifecycleNode& node) override {
        auto sn = std::make_shared<SessionType>(node);
        // init parameters (must be before publishers/subscribers for QoS param refs)
        sn->param_listener = std::make_shared<ParamListener>(sn->node.shared_from_this());
        sn->params = sn->param_listener->get_params();
        // init services
        sn->services.trigger_service = cake::create_service<std_srvs::srv::Trigger>(sn, "/trigger_service");
        sn->services.compute = cake::create_service<example_interfaces::srv::AddTwoInts>(sn, "compute");
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

} // namespace test_package::services_default_qos
