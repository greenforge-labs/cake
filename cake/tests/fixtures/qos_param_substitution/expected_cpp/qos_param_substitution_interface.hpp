// auto-generated DO NOT EDIT

#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <cake/base_node.hpp>
#include <cake/session.hpp>
#include <cake/publisher.hpp>
#include <cake/subscriber.hpp>
#include <cake/qos_helpers.hpp>
#include <test_package/qos_param_substitution_parameters.hpp>

namespace test_package::qos_param_substitution {

template <typename SessionType> struct QosParamSubstitutionPublishers {
    std::shared_ptr<cake::Publisher<std_msgs::msg::String, SessionType>> processed_data;
};

template <typename SessionType> struct QosParamSubstitutionSubscribers {
    std::shared_ptr<cake::Subscriber<sensor_msgs::msg::LaserScan, SessionType>> sensor_data;
};

template <typename SessionType> struct QosParamSubstitutionServices {};

template <typename SessionType> struct QosParamSubstitutionServiceClients {};

template <typename SessionType> struct QosParamSubstitutionActions {};

template <typename SessionType> struct QosParamSubstitutionActionClients {};

template <typename DerivedSessionType> struct QosParamSubstitutionSession : cake::Session {
    using cake::Session::Session;
    QosParamSubstitutionPublishers<DerivedSessionType> publishers;
    QosParamSubstitutionSubscribers<DerivedSessionType> subscribers;
    QosParamSubstitutionServices<DerivedSessionType> services;
    QosParamSubstitutionServiceClients<DerivedSessionType> service_clients;
    QosParamSubstitutionActions<DerivedSessionType> actions;
    QosParamSubstitutionActionClients<DerivedSessionType> action_clients;
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
class QosParamSubstitutionBase : public cake::BaseNode<"qos_param_substitution", SessionType, extend_options> {
    static_assert(
        std::is_base_of_v<QosParamSubstitutionSession<SessionType>, SessionType>, "SessionType must be a child of QosParamSubstitutionSession"
    );

  public:
    explicit QosParamSubstitutionBase(const rclcpp::NodeOptions &options)
        : cake::BaseNode<"qos_param_substitution", SessionType, extend_options>(options) {}

  protected:
    std::shared_ptr<SessionType> create_session(rclcpp_lifecycle::LifecycleNode& node) override {
        auto sn = std::make_shared<SessionType>(node);
        // init parameters (must be before publishers/subscribers for QoS param refs)
        sn->param_listener = std::make_shared<ParamListener>(sn->node.shared_from_this());
        sn->params = sn->param_listener->get_params();

        // init publishers
        sn->publishers.processed_data = cake::create_publisher<std_msgs::msg::String>(sn, "/processed_data", rclcpp::QoS(sn->params.output_queue_depth).reliable());
        // init subscribers
        sn->subscribers.sensor_data = cake::create_subscriber<sensor_msgs::msg::LaserScan>(sn, "/sensor_data", rclcpp::QoS(sn->params.sensor_queue_depth).reliability(cake::to_reliability(sn->params.sensor_reliability)).durability(cake::to_durability(sn->params.sensor_durability)).deadline(rclcpp::Duration::from_nanoseconds(sn->params.sensor_deadline_ms * 1000000LL)).liveliness(cake::to_liveliness(sn->params.sensor_liveliness)));
        return sn;
    }

    void activate_entities(std::shared_ptr<SessionType> sn) override {
        sn->publishers.processed_data->activate();
        for (auto &t : sn->timers) { t->reset(); }
    }

    void deactivate_entities(std::shared_ptr<SessionType> sn) override {
        for (auto &t : sn->timers) { t->cancel(); }
        if (sn->publishers.processed_data) { sn->publishers.processed_data->deactivate(); }
    }

    CallbackReturn user_on_configure(std::shared_ptr<SessionType> sn) override { return on_configure_func(sn); }
    CallbackReturn user_on_activate(std::shared_ptr<SessionType> sn) override { return on_activate_func(sn); }
    CallbackReturn user_on_deactivate(std::shared_ptr<SessionType> sn) override { return on_deactivate_func(sn); }
    CallbackReturn user_on_cleanup(std::shared_ptr<SessionType> sn) override { return on_cleanup_func(sn); }
    void user_on_shutdown(std::shared_ptr<SessionType> sn) override { on_shutdown_func(sn); }
};

} // namespace test_package::qos_param_substitution
