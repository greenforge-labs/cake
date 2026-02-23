// auto-generated DO NOT EDIT

#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <cake/base_node.hpp>
#include <cake/session.hpp>
#include <cake/publisher.hpp>
#include <cake/subscriber.hpp>
#include <test_package/no_node_section_parameters.hpp>

namespace test_package::no_node_section {

template <typename SessionType> struct NoNodeSectionPublishers {
    std::shared_ptr<cake::Publisher<std_msgs::msg::String, SessionType>> status;
};

template <typename SessionType> struct NoNodeSectionSubscribers {
    std::shared_ptr<cake::Subscriber<std_msgs::msg::Bool, SessionType>> input;
};

template <typename SessionType> struct NoNodeSectionServices {};

template <typename SessionType> struct NoNodeSectionServiceClients {};

template <typename SessionType> struct NoNodeSectionActions {};

template <typename SessionType> struct NoNodeSectionActionClients {};

template <typename DerivedSessionType> struct NoNodeSectionSession : cake::Session {
    using cake::Session::Session;
    NoNodeSectionPublishers<DerivedSessionType> publishers;
    NoNodeSectionSubscribers<DerivedSessionType> subscribers;
    NoNodeSectionServices<DerivedSessionType> services;
    NoNodeSectionServiceClients<DerivedSessionType> service_clients;
    NoNodeSectionActions<DerivedSessionType> actions;
    NoNodeSectionActionClients<DerivedSessionType> action_clients;
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
class NoNodeSectionBase : public cake::BaseNode<"no_node_section", SessionType, extend_options> {
    static_assert(
        std::is_base_of_v<NoNodeSectionSession<SessionType>, SessionType>, "SessionType must be a child of NoNodeSectionSession"
    );

  public:
    explicit NoNodeSectionBase(const rclcpp::NodeOptions &options)
        : cake::BaseNode<"no_node_section", SessionType, extend_options>(options) {}

  protected:
    std::shared_ptr<SessionType> create_session(rclcpp_lifecycle::LifecycleNode& node) override {
        auto sn = std::make_shared<SessionType>(node);
        // init parameters (must be before publishers/subscribers for QoS param refs)
        sn->param_listener = std::make_shared<ParamListener>(sn->node.shared_from_this());
        sn->params = sn->param_listener->get_params();

        // init publishers
        sn->publishers.status = cake::create_publisher<std_msgs::msg::String>(sn, "/status", rclcpp::QoS(10).reliable());
        // init subscribers
        sn->subscribers.input = cake::create_subscriber<std_msgs::msg::Bool>(sn, "/input", rclcpp::QoS(5).best_effort());
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

} // namespace test_package::no_node_section
