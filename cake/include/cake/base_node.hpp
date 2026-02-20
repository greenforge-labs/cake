#pragma once

#include <memory>
#include <type_traits>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "context.hpp"
#include "fixed_string.hpp"

namespace cake {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

template <
    fixed_string node_name,
    typename ContextType,
    auto extend_options = [](rclcpp::NodeOptions options) { return options; }>
class BaseNode {
    static_assert(std::is_base_of_v<Context, ContextType>, "ContextType must derive from cake::Context");

  public:
    explicit BaseNode(const rclcpp::NodeOptions &options)
        : node_(std::make_shared<rclcpp_lifecycle::LifecycleNode>(node_name.c_str(), extend_options(options))) {
        auto ctx = std::make_shared<ContextType>();
        reset_context(ctx);

        node_->register_on_configure([this, ctx](const auto &) { return handle_configure(ctx); });
        node_->register_on_activate([this, ctx](const auto &) { return handle_activate(ctx); });
        node_->register_on_deactivate([this, ctx](const auto &) { return handle_deactivate(ctx); });
        node_->register_on_cleanup([this, ctx](const auto &) { return handle_cleanup(ctx); });
        node_->register_on_shutdown([this, ctx](const auto &) { return handle_shutdown(ctx); });
        node_->register_on_error([this, ctx](const auto &) { return handle_error(ctx); });
    }

    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() const {
        return node_->get_node_base_interface();
    }

  private:
    void reset_context(std::shared_ptr<ContextType> ctx) {
        *ctx = ContextType{};
        ctx->node = node_;
    }

    CallbackReturn handle_configure(std::shared_ptr<ContextType> ctx) {
        create_entities(ctx);
        auto result = user_on_configure(ctx);
        if (result == CallbackReturn::FAILURE) {
            reset_context(ctx);
        }
        return result;
    }

    CallbackReturn handle_activate(std::shared_ptr<ContextType> ctx) {
        auto result = user_on_activate(ctx);
        if (result != CallbackReturn::SUCCESS)
            return result;
        activate_entities(ctx);
        return result;
    }

    CallbackReturn handle_deactivate(std::shared_ptr<ContextType> ctx) {
        auto result = user_on_deactivate(ctx);
        if (result != CallbackReturn::SUCCESS)
            return result;
        deactivate_entities(ctx);
        return result;
    }

    CallbackReturn handle_cleanup(std::shared_ptr<ContextType> ctx) {
        auto result = user_on_cleanup(ctx);
        if (result == CallbackReturn::SUCCESS) {
            reset_context(ctx);
        }
        return result;
    }

    CallbackReturn handle_shutdown(std::shared_ptr<ContextType> ctx) {
        user_on_shutdown(ctx);
        deactivate_entities(ctx);
        reset_context(ctx);
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn handle_error(std::shared_ptr<ContextType> ctx) {
        deactivate_entities(ctx);
        reset_context(ctx);
        return CallbackReturn::FAILURE;
    }

  protected:
    virtual void create_entities(std::shared_ptr<ContextType> ctx) {}
    virtual void activate_entities(std::shared_ptr<ContextType> ctx) {}
    virtual void deactivate_entities(std::shared_ptr<ContextType> ctx) {}

    virtual CallbackReturn user_on_configure(std::shared_ptr<ContextType> ctx) { return CallbackReturn::SUCCESS; }
    virtual CallbackReturn user_on_activate(std::shared_ptr<ContextType> ctx) { return CallbackReturn::SUCCESS; }
    virtual CallbackReturn user_on_deactivate(std::shared_ptr<ContextType> ctx) { return CallbackReturn::SUCCESS; }
    virtual CallbackReturn user_on_cleanup(std::shared_ptr<ContextType> ctx) { return CallbackReturn::SUCCESS; }
    virtual void user_on_shutdown(std::shared_ptr<ContextType> ctx) {}

    rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
};

} // namespace cake
