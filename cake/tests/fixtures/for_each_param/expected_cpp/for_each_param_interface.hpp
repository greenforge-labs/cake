// auto-generated DO NOT EDIT

#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <cake/base_node.hpp>
#include <cake/session.hpp>
#include <cake/publisher.hpp>
#include <cake/subscriber.hpp>
#include <string>
#include <unordered_map>
#include <test_package/for_each_param_parameters.hpp>

namespace test_package::for_each_param {

template <typename SessionType> struct ForEachParamPublishers {
    std::shared_ptr<cake::Publisher<std_msgs::msg::String, SessionType>> status;
};

template <typename SessionType> struct ForEachParamSubscribers {
    std::unordered_map<std::string, std::shared_ptr<cake::Subscriber<std_msgs::msg::String, SessionType>>> node_states;
};

template <typename SessionType> struct ForEachParamServices {};

template <typename SessionType> struct ForEachParamServiceClients {
    std::unordered_map<std::string, rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr> change_state_clients;
};

template <typename SessionType> struct ForEachParamActions {};

template <typename SessionType> struct ForEachParamActionClients {};

template <typename DerivedSessionType> struct ForEachParamSession : cake::Session {
    using cake::Session::Session;
    ForEachParamPublishers<DerivedSessionType> publishers;
    ForEachParamSubscribers<DerivedSessionType> subscribers;
    ForEachParamServices<DerivedSessionType> services;
    ForEachParamServiceClients<DerivedSessionType> service_clients;
    ForEachParamActions<DerivedSessionType> actions;
    ForEachParamActionClients<DerivedSessionType> action_clients;
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
class ForEachParamBase : public cake::BaseNode<"for_each_param", SessionType, extend_options> {
    static_assert(
        std::is_base_of_v<ForEachParamSession<SessionType>, SessionType>, "SessionType must be a child of ForEachParamSession"
    );

  public:
    explicit ForEachParamBase(const rclcpp::NodeOptions &options)
        : cake::BaseNode<"for_each_param", SessionType, extend_options>(options) {}

  protected:
    std::shared_ptr<SessionType> create_session(rclcpp_lifecycle::LifecycleNode& node) override {
        auto sn = std::make_shared<SessionType>(node);
        // init parameters (must be before publishers/subscribers for QoS param refs)
        sn->param_listener = std::make_shared<ParamListener>(sn->node.shared_from_this());
        sn->params = sn->param_listener->get_params();

        // init publishers
        sn->publishers.status = cake::create_publisher<std_msgs::msg::String>(sn, "/robot/" + sn->params.robot_id + "/status", rclcpp::QoS(10).reliable());
        // init subscribers
        for (const auto& key : sn->params.managed_nodes) {
            sn->subscribers.node_states[key] = cake::create_subscriber<std_msgs::msg::String>(sn, "/" + key + "/state", rclcpp::QoS(10).reliable());
        }
        // init service clients
        for (const auto& key : sn->params.managed_nodes) {
            sn->service_clients.change_state_clients[key] = sn->node.template create_client<lifecycle_msgs::srv::ChangeState>("/" + key + "/change_state");
        }
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

} // namespace test_package::for_each_param
