// auto-generated DO NOT EDIT

#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <example_interfaces/srv/add_two_ints.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <cake/base_node.hpp>
#include <cake/session.hpp>
#include <test_package/service_clients_only_parameters.hpp>

namespace test_package::service_clients_only {

template <typename SessionType> struct ServiceClientsOnlyPublishers {};

template <typename SessionType> struct ServiceClientsOnlySubscribers {};

template <typename SessionType> struct ServiceClientsOnlyServices {};

template <typename SessionType> struct ServiceClientsOnlyServiceClients {
    rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr add_two_ints;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr trigger_service;
};

template <typename SessionType> struct ServiceClientsOnlyActions {};

template <typename SessionType> struct ServiceClientsOnlyActionClients {};

template <typename DerivedSessionType> struct ServiceClientsOnlySession : cake::Session {
    using cake::Session::Session;
    ServiceClientsOnlyPublishers<DerivedSessionType> publishers;
    ServiceClientsOnlySubscribers<DerivedSessionType> subscribers;
    ServiceClientsOnlyServices<DerivedSessionType> services;
    ServiceClientsOnlyServiceClients<DerivedSessionType> service_clients;
    ServiceClientsOnlyActions<DerivedSessionType> actions;
    ServiceClientsOnlyActionClients<DerivedSessionType> action_clients;
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
class ServiceClientsOnlyBase : public cake::BaseNode<"service_clients_only", SessionType, extend_options> {
    static_assert(
        std::is_base_of_v<ServiceClientsOnlySession<SessionType>, SessionType>, "SessionType must be a child of ServiceClientsOnlySession"
    );

  public:
    explicit ServiceClientsOnlyBase(const rclcpp::NodeOptions &options)
        : cake::BaseNode<"service_clients_only", SessionType, extend_options>(options) {}

  protected:
    std::shared_ptr<SessionType> create_session(rclcpp_lifecycle::LifecycleNode& node) override {
        auto sn = std::make_shared<SessionType>(node);
        // init parameters (must be before publishers/subscribers for QoS param refs)
        sn->param_listener = std::make_shared<ParamListener>(sn->node.shared_from_this());
        sn->params = sn->param_listener->get_params();
        // init service clients
        sn->service_clients.add_two_ints = sn->node.template create_client<example_interfaces::srv::AddTwoInts>("/add_two_ints", rclcpp::ServicesQoS(), this->client_callback_group());
        sn->service_clients.trigger_service = sn->node.template create_client<std_srvs::srv::Trigger>("trigger_service", rclcpp::ServicesQoS(), this->client_callback_group());
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

} // namespace test_package::service_clients_only
