// auto-generated DO NOT EDIT

#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <cake/base_node.hpp>
#include <cake/session.hpp>
#include <test_package/empty_node_parameters.hpp>

namespace test_package::empty_node {

template <typename SessionType> struct EmptyNodePublishers {};

template <typename SessionType> struct EmptyNodeSubscribers {};

template <typename SessionType> struct EmptyNodeServices {};

template <typename SessionType> struct EmptyNodeServiceClients {};

template <typename SessionType> struct EmptyNodeActions {};

template <typename SessionType> struct EmptyNodeActionClients {};

template <typename DerivedSessionType> struct EmptyNodeSession : cake::Session {
    using cake::Session::Session;
    EmptyNodePublishers<DerivedSessionType> publishers;
    EmptyNodeSubscribers<DerivedSessionType> subscribers;
    EmptyNodeServices<DerivedSessionType> services;
    EmptyNodeServiceClients<DerivedSessionType> service_clients;
    EmptyNodeActions<DerivedSessionType> actions;
    EmptyNodeActionClients<DerivedSessionType> action_clients;
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
class EmptyNodeBase : public cake::BaseNode<"empty_node", SessionType, extend_options> {
    static_assert(
        std::is_base_of_v<EmptyNodeSession<SessionType>, SessionType>, "SessionType must be a child of EmptyNodeSession"
    );

  public:
    explicit EmptyNodeBase(const rclcpp::NodeOptions &options)
        : cake::BaseNode<"empty_node", SessionType, extend_options>(options) {}

  protected:
    std::shared_ptr<SessionType> create_session(rclcpp_lifecycle::LifecycleNode& node) override {
        auto sn = std::make_shared<SessionType>(node);
        // init parameters (must be before publishers/subscribers for QoS param refs)
        sn->param_listener = std::make_shared<ParamListener>(sn->node.shared_from_this());
        sn->params = sn->param_listener->get_params();
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

} // namespace test_package::empty_node
