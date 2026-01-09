// auto-generated DO NOT EDIT

#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/action/fibonacci.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <cake/base_node.hpp>
#include <cake/context.hpp>
#include <cake/publisher.hpp>
#include <cake/subscriber.hpp>
#include <cake/service.hpp>
#include <cake/action_server.hpp>
#include <test_package/param_substitution_parameters.hpp>

namespace test_package::param_substitution {

namespace detail {
inline void apply_reliability(rclcpp::QoS& qos, const std::string& value) {
    if (value == "reliable") { qos.reliable(); }
    else if (value == "best_effort") { qos.best_effort(); }
    else { throw std::runtime_error("Invalid reliability policy: " + value); }
}

inline void apply_durability(rclcpp::QoS& qos, const std::string& value) {
    if (value == "volatile") { qos.durability_volatile(); }
    else if (value == "transient_local") { qos.transient_local(); }
    else { throw std::runtime_error("Invalid durability policy: " + value); }
}

inline void apply_history(rclcpp::QoS& qos, const std::string& value, int depth) {
    if (value == "keep_last") { qos.keep_last(depth); }
    else if (value == "keep_all") { qos.keep_all(); }
    else { throw std::runtime_error("Invalid history policy: " + value); }
}

inline void apply_liveliness(rclcpp::QoS& qos, const std::string& value) {
    if (value == "automatic") { qos.liveliness(rclcpp::LivelinessPolicy::Automatic); }
    else if (value == "manual_by_topic") { qos.liveliness(rclcpp::LivelinessPolicy::ManualByTopic); }
    else { throw std::runtime_error("Invalid liveliness policy: " + value); }
}
} // namespace detail

template <typename ContextType> struct ParamSubstitutionPublishers {
    std::shared_ptr<cake::Publisher<std_msgs::msg::String, ContextType>> topic_prefix;
    std::shared_ptr<cake::Publisher<std_msgs::msg::String, ContextType>> topic_prefix_robot_name_status;
    std::shared_ptr<cake::Publisher<geometry_msgs::msg::Twist, ContextType>> cmd_vel;
    std::shared_ptr<cake::Publisher<std_msgs::msg::Float32, ContextType>> sensor_data;
    std::shared_ptr<cake::Publisher<std_msgs::msg::String, ContextType>> critical_data;
};

template <typename ContextType> struct ParamSubstitutionSubscribers {
    std::shared_ptr<cake::Subscriber<std_msgs::msg::String, ContextType>> topic_prefix_robot_name_command;
};

template <typename ContextType> struct ParamSubstitutionServices {
    std::shared_ptr<cake::Service<std_srvs::srv::Trigger, ContextType>> topic_prefix_robot_name_reset;
};

template <typename ContextType> struct ParamSubstitutionServiceClients {};

template <typename ContextType> struct ParamSubstitutionActions {
    std::shared_ptr<cake::SingleGoalActionServer<example_interfaces::action::Fibonacci>> topic_prefix_robot_name_navigate;
};

template <typename ContextType> struct ParamSubstitutionActionClients {};

template <typename DerivedContextType> struct ParamSubstitutionContext : cake::Context {
    ParamSubstitutionPublishers<DerivedContextType> publishers;
    ParamSubstitutionSubscribers<DerivedContextType> subscribers;
    ParamSubstitutionServices<DerivedContextType> services;
    ParamSubstitutionServiceClients<DerivedContextType> service_clients;
    ParamSubstitutionActions<DerivedContextType> actions;
    ParamSubstitutionActionClients<DerivedContextType> action_clients;
    std::shared_ptr<ParamListener> param_listener;
    Params params;
};


template <
    typename ContextType,
    auto init_func,
    auto extend_options = [](rclcpp::NodeOptions options) { return options; }>
class ParamSubstitutionBase : public cake::BaseNode<"param_substitution", extend_options> {
  public:
    explicit ParamSubstitutionBase(const rclcpp::NodeOptions &options) : cake::BaseNode<"param_substitution", extend_options>(options) {
        static_assert(
            std::is_base_of_v<ParamSubstitutionContext<ContextType>, ContextType>, "ContextType must be a child of ParamSubstitutionContext"
        );

        // init context
        auto ctx = std::make_shared<ContextType>();
        ctx->node = this->node_;

        // init parameters (before entities to support ${params.X} substitutions)
        ctx->param_listener = std::make_shared<ParamListener>(ctx->node);
        ctx->params = ctx->param_listener->get_params();

        // init publishers
        ctx->publishers.topic_prefix = cake::create_publisher<std_msgs::msg::String>(ctx, ctx->params.topic_prefix, 10);
        ctx->publishers.topic_prefix_robot_name_status = cake::create_publisher<std_msgs::msg::String>(ctx, ctx->params.topic_prefix + std::string("/") + ctx->params.robot_name + std::string("/status"), ctx->params.qos_depth);
        ctx->publishers.cmd_vel = cake::create_publisher<geometry_msgs::msg::Twist>(ctx, "/cmd_vel", rclcpp::QoS(ctx->params.qos_depth).reliable());
        ctx->publishers.sensor_data = cake::create_publisher<std_msgs::msg::Float32>(ctx, "/sensor_data", [&]() { rclcpp::QoS qos(ctx->params.qos_depth); detail::apply_reliability(qos, ctx->params.reliability_mode); return qos; }());
        ctx->publishers.critical_data = cake::create_publisher<std_msgs::msg::String>(ctx, "/critical_data", rclcpp::QoS(10).deadline(rclcpp::Duration(ctx->params.deadline_sec, 0)));
        // init subscribers
        ctx->subscribers.topic_prefix_robot_name_command = cake::create_subscriber<std_msgs::msg::String>(ctx, ctx->params.topic_prefix + std::string("/") + ctx->params.robot_name + std::string("/command"), ctx->params.qos_depth);
        // init services
        ctx->services.topic_prefix_robot_name_reset = cake::create_service<std_srvs::srv::Trigger>(ctx, ctx->params.topic_prefix + std::string("/") + ctx->params.robot_name + std::string("/reset"));
        // init actions
        ctx->actions.topic_prefix_robot_name_navigate = cake::create_single_goal_action_server<example_interfaces::action::Fibonacci>(ctx, ctx->params.topic_prefix + std::string("/") + ctx->params.robot_name + std::string("/navigate"));
        init_func(ctx);
    }
};

} // namespace test_package::param_substitution
