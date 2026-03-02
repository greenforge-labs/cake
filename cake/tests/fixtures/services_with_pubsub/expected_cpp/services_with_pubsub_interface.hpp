// auto-generated DO NOT EDIT

#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <example_interfaces/srv/add_two_ints.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <cake/base_node.hpp>
#include <cake/session.hpp>
#include <cake/publisher.hpp>
#include <cake/subscriber.hpp>
#include <cake/default_qos_handlers.hpp>
#include <cake/service.hpp>
#include <test_package/services_with_pubsub_parameters.hpp>

namespace test_package::services_with_pubsub {

template <typename SessionType> struct ServicesWithPubsubPublishers {
    std::shared_ptr<cake::Publisher<std_msgs::msg::String, SessionType>> status;
};

template <typename SessionType> struct ServicesWithPubsubSubscribers {
    std::shared_ptr<cake::Subscriber<std_msgs::msg::String, SessionType>> command;
};

template <typename SessionType> struct ServicesWithPubsubServices {
    std::shared_ptr<cake::Service<std_srvs::srv::Trigger, SessionType>> reset;
    std::shared_ptr<cake::Service<example_interfaces::srv::AddTwoInts, SessionType>> compute;
};

template <typename SessionType> struct ServicesWithPubsubServiceClients {};

template <typename SessionType> struct ServicesWithPubsubActions {};

template <typename SessionType> struct ServicesWithPubsubActionClients {};

template <typename DerivedSessionType> struct ServicesWithPubsubSession : cake::Session {
    using cake::Session::Session;
    ServicesWithPubsubPublishers<DerivedSessionType> publishers;
    ServicesWithPubsubSubscribers<DerivedSessionType> subscribers;
    ServicesWithPubsubServices<DerivedSessionType> services;
    ServicesWithPubsubServiceClients<DerivedSessionType> service_clients;
    ServicesWithPubsubActions<DerivedSessionType> actions;
    ServicesWithPubsubActionClients<DerivedSessionType> action_clients;
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
class ServicesWithPubsubBase : public cake::BaseNode<"services_with_pubsub", SessionType, extend_options> {
    static_assert(
        std::is_base_of_v<ServicesWithPubsubSession<SessionType>, SessionType>, "SessionType must be a child of ServicesWithPubsubSession"
    );

  public:
    explicit ServicesWithPubsubBase(const rclcpp::NodeOptions &options)
        : cake::BaseNode<"services_with_pubsub", SessionType, extend_options>(options) {}

  protected:
    std::shared_ptr<SessionType> create_session(rclcpp_lifecycle::LifecycleNode& node) override {
        auto sn = std::make_shared<SessionType>(node);
        // init parameters (must be before publishers/subscribers for QoS param refs)
        sn->param_listener = std::make_shared<ParamListener>(sn->node.shared_from_this());
        sn->params = sn->param_listener->get_params();

        // init publishers
        sn->publishers.status = cake::create_publisher<std_msgs::msg::String>(sn, "/status", rclcpp::QoS(10).reliable());
        // init subscribers
        sn->subscribers.command = cake::create_subscriber<std_msgs::msg::String>(sn, "/command", rclcpp::QoS(5).best_effort());
        cake::attach_default_qos_handlers(sn->subscribers.command);
        // init services
        sn->services.reset = cake::create_service<std_srvs::srv::Trigger>(sn, "/reset");
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

} // namespace test_package::services_with_pubsub
