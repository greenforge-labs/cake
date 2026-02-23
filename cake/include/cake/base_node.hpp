#pragma once

#include <memory>
#include <type_traits>

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
};

} // namespace cake
