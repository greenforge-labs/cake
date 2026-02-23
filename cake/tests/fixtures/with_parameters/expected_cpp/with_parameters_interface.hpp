// auto-generated DO NOT EDIT

#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/string.hpp>
#include <cake/base_node.hpp>
#include <cake/session.hpp>
#include <cake/publisher.hpp>
#include <test_package/with_parameters_parameters.hpp>

namespace test_package::with_parameters {

template <typename SessionType> struct WithParametersPublishers {
    std::shared_ptr<cake::Publisher<std_msgs::msg::String, SessionType>> status;
};

template <typename SessionType> struct WithParametersSubscribers {};

template <typename SessionType> struct WithParametersServices {};

template <typename SessionType> struct WithParametersServiceClients {};

template <typename SessionType> struct WithParametersActions {};

template <typename SessionType> struct WithParametersActionClients {};

template <typename DerivedSessionType> struct WithParametersSession : cake::Session {
    using cake::Session::Session;
    WithParametersPublishers<DerivedSessionType> publishers;
    WithParametersSubscribers<DerivedSessionType> subscribers;
    WithParametersServices<DerivedSessionType> services;
    WithParametersServiceClients<DerivedSessionType> service_clients;
    WithParametersActions<DerivedSessionType> actions;
    WithParametersActionClients<DerivedSessionType> action_clients;
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
class WithParametersBase : public cake::BaseNode<"with_parameters", SessionType, extend_options> {
    static_assert(
        std::is_base_of_v<WithParametersSession<SessionType>, SessionType>, "SessionType must be a child of WithParametersSession"
    );

  public:
    explicit WithParametersBase(const rclcpp::NodeOptions &options)
        : cake::BaseNode<"with_parameters", SessionType, extend_options>(options) {}

  protected:
    std::shared_ptr<SessionType> create_session(rclcpp_lifecycle::LifecycleNode& node) override {
        auto sn = std::make_shared<SessionType>(node);
        // init parameters (must be before publishers/subscribers for QoS param refs)
        sn->param_listener = std::make_shared<ParamListener>(sn->node.shared_from_this());
        sn->params = sn->param_listener->get_params();

        // init publishers
        sn->publishers.status = cake::create_publisher<std_msgs::msg::String>(sn, "/status", rclcpp::QoS(10).reliable());
        return sn;
    }

    void activate_entities(std::shared_ptr<SessionType> sn) override {
        sn->publishers.status->activate();
        for (auto &t : sn->timers) { t->reset(); }
    }

    void deactivate_entities(std::shared_ptr<SessionType> sn) override {
        for (auto &t : sn->timers) { t->cancel(); }
        if (sn->publishers.status) { sn->publishers.status->deactivate(); }
    }

    CallbackReturn user_on_configure(std::shared_ptr<SessionType> sn) override { return on_configure_func(sn); }
    CallbackReturn user_on_activate(std::shared_ptr<SessionType> sn) override { return on_activate_func(sn); }
    CallbackReturn user_on_deactivate(std::shared_ptr<SessionType> sn) override { return on_deactivate_func(sn); }
    CallbackReturn user_on_cleanup(std::shared_ptr<SessionType> sn) override { return on_cleanup_func(sn); }
    void user_on_shutdown(std::shared_ptr<SessionType> sn) override { on_shutdown_func(sn); }
};

} // namespace test_package::with_parameters
