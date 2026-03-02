#pragma once

#include <memory>
#include <type_traits>

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "fixed_string.hpp"
#include "session.hpp"

namespace cake {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

template <
    fixed_string node_name,
    typename SessionType,
    auto extend_options = [](rclcpp::NodeOptions options) { return options; }>
class BaseNode {
    static_assert(std::is_base_of_v<Session, SessionType>, "SessionType must derive from cake::Session");

  public:
    explicit BaseNode(const rclcpp::NodeOptions &options)
        : node_(std::make_shared<rclcpp_lifecycle::LifecycleNode>(node_name.c_str(), extend_options(options))) {
        node_->register_on_configure([this](const auto &) { return handle_configure(); });
        node_->register_on_activate([this](const auto &) { return handle_activate(); });
        node_->register_on_deactivate([this](const auto &) { return handle_deactivate(); });
        node_->register_on_cleanup([this](const auto &) { return handle_cleanup(); });
        node_->register_on_shutdown([this](const auto &) { return handle_shutdown(); });
        node_->register_on_error([this](const auto &) { return handle_error(); });

        // State heartbeat publisher + timer (always active, not lifecycle-managed).
        auto state_qos = rclcpp::QoS(1).reliable().transient_local().deadline(std::chrono::milliseconds(100));
        auto node_params = node_->get_node_parameters_interface();
        auto node_topics = node_->get_node_topics_interface();
        state_pub_ =
            rclcpp::create_publisher<lifecycle_msgs::msg::State>(node_params, node_topics, "~/state", state_qos);
        state_timer_ = node_->create_wall_timer(std::chrono::milliseconds(100), [this]() {
            if (state_pub_->get_subscription_count() == 0) {
                return;
            }
            lifecycle_msgs::msg::State msg;
            auto state = node_->get_current_state();
            msg.id = state.id();
            msg.label = state.label();
            state_pub_->publish(msg);
        });

        node_->declare_parameter("autostart", true);
        if (node_->get_parameter("autostart").as_bool()) {
            using lifecycle_msgs::msg::State;
            autostart_timer_ = node_->create_wall_timer(std::chrono::seconds(0), [this]() {
                autostart_timer_->cancel();
                if (node_->configure().id() != State::PRIMARY_STATE_INACTIVE) {
                    RCLCPP_ERROR(node_->get_logger(), "Autostart failed to configure");
                    return;
                }
                if (node_->activate().id() != State::PRIMARY_STATE_ACTIVE) {
                    RCLCPP_ERROR(node_->get_logger(), "Autostart failed to activate");
                }
            });
        }
    }

    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() const {
        return node_->get_node_base_interface();
    }

  private:
    CallbackReturn handle_configure() {
        session_ = create_session(*node_);
        auto result = user_on_configure(session_);
        if (result == CallbackReturn::FAILURE) {
            session_.reset();
        }
        return result;
    }

    CallbackReturn handle_activate() {
        auto result = user_on_activate(session_);
        if (result != CallbackReturn::SUCCESS) {
            return result;
        }
        activate_entities(session_);
        return result;
    }

    CallbackReturn handle_deactivate() {
        auto result = user_on_deactivate(session_);
        if (result != CallbackReturn::SUCCESS) {
            return result;
        }
        deactivate_entities(session_);
        return result;
    }

    CallbackReturn handle_cleanup() {
        auto result = user_on_cleanup(session_);
        if (result == CallbackReturn::SUCCESS) {
            session_.reset();
        }
        return result;
    }

    CallbackReturn handle_shutdown() {
        if (!session_) {
            return CallbackReturn::SUCCESS;
        }
        user_on_shutdown(session_);
        session_.reset();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn handle_error() {
        if (!session_) {
            return CallbackReturn::FAILURE;
        }
        session_.reset();
        // always return failure so that we end in Finalized (our assertion is errors are unrecoverable)
        return CallbackReturn::FAILURE;
    }

  protected:
    virtual std::shared_ptr<SessionType> create_session(rclcpp_lifecycle::LifecycleNode &node) = 0;
    virtual void activate_entities(std::shared_ptr<SessionType> /*sn*/) {}
    virtual void deactivate_entities(std::shared_ptr<SessionType> /*sn*/) {}

    virtual CallbackReturn user_on_configure(std::shared_ptr<SessionType> /*sn*/) { return CallbackReturn::SUCCESS; }
    virtual CallbackReturn user_on_activate(std::shared_ptr<SessionType> /*sn*/) { return CallbackReturn::SUCCESS; }
    virtual CallbackReturn user_on_deactivate(std::shared_ptr<SessionType> /*sn*/) { return CallbackReturn::SUCCESS; }
    virtual CallbackReturn user_on_cleanup(std::shared_ptr<SessionType> /*sn*/) { return CallbackReturn::SUCCESS; }
    virtual void user_on_shutdown(std::shared_ptr<SessionType> /*sn*/) {}

    rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
    std::shared_ptr<SessionType> session_;
    rclcpp::TimerBase::SharedPtr autostart_timer_;
    rclcpp::Publisher<lifecycle_msgs::msg::State>::SharedPtr state_pub_;
    rclcpp::TimerBase::SharedPtr state_timer_;
};

} // namespace cake
