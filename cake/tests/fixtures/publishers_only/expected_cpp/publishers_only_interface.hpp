// auto-generated DO NOT EDIT

#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <cake/base_node.hpp>
#include <cake/session.hpp>
#include <cake/publisher.hpp>
#include <test_package/publishers_only_parameters.hpp>

namespace test_package::publishers_only {

template <typename SessionType> struct PublishersOnlyPublishers {
    std::shared_ptr<cake::Publisher<std_msgs::msg::String, SessionType>> status;
    std::shared_ptr<cake::Publisher<std_msgs::msg::Int32, SessionType>> counter;
};

template <typename SessionType> struct PublishersOnlySubscribers {};

template <typename SessionType> struct PublishersOnlyServices {};

template <typename SessionType> struct PublishersOnlyServiceClients {};

template <typename SessionType> struct PublishersOnlyActions {};

template <typename SessionType> struct PublishersOnlyActionClients {};

template <typename DerivedSessionType> struct PublishersOnlySession : cake::Session {
    using cake::Session::Session;
    PublishersOnlyPublishers<DerivedSessionType> publishers;
    PublishersOnlySubscribers<DerivedSessionType> subscribers;
    PublishersOnlyServices<DerivedSessionType> services;
    PublishersOnlyServiceClients<DerivedSessionType> service_clients;
    PublishersOnlyActions<DerivedSessionType> actions;
    PublishersOnlyActionClients<DerivedSessionType> action_clients;
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
class PublishersOnlyBase : public cake::BaseNode<"publishers_only", SessionType, extend_options> {
    static_assert(
        std::is_base_of_v<PublishersOnlySession<SessionType>, SessionType>, "SessionType must be a child of PublishersOnlySession"
    );

  public:
    explicit PublishersOnlyBase(const rclcpp::NodeOptions &options)
        : cake::BaseNode<"publishers_only", SessionType, extend_options>(options) {}

  protected:
    std::shared_ptr<SessionType> create_session(rclcpp_lifecycle::LifecycleNode& node) override {
        auto sn = std::make_shared<SessionType>(node);
        // init parameters (must be before publishers/subscribers for QoS param refs)
        sn->param_listener = std::make_shared<ParamListener>(sn->node.shared_from_this());
        sn->params = sn->param_listener->get_params();

        // init publishers
        sn->publishers.status = cake::create_publisher<std_msgs::msg::String>(sn, "status", rclcpp::QoS(10).reliable());
        sn->publishers.counter = cake::create_publisher<std_msgs::msg::Int32>(sn, "counter", rclcpp::QoS(5).best_effort());
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

} // namespace test_package::publishers_only
