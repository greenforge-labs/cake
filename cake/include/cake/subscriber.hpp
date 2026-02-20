#pragma once

#include <functional>
#include <memory>
#include <type_traits>

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>

#include "context.hpp"

namespace cake {

template <typename MessageT, typename ContextType> class Subscriber {
    static_assert(std::is_base_of_v<Context, ContextType>, "ContextType must derive from cake::Context");

  public:
    explicit Subscriber(std::shared_ptr<ContextType> context, const std::string &topic_name, const rclcpp::QoS &qos)
        : context_(context) {
        set_callback([topic_name](auto ctx, auto /*msg*/) {
            RCLCPP_WARN(
                ctx->node->get_logger(),
                "Subscriber '%s' received message but no callback configured. Call set_callback().",
                topic_name.c_str()
            );
        });

        // Register noop callbacks that delegate to optional user callbacks
        rclcpp::SubscriptionOptions options;
        options.event_callbacks.deadline_callback = [this](rclcpp::QOSDeadlineRequestedInfo &event) {
            if (auto ctx = context_.lock()) {
                if (deadline_callback_) {
                    deadline_callback_(ctx, event);
                }
            }
        };
        options.event_callbacks.liveliness_callback = [this](rclcpp::QOSLivelinessChangedInfo &event) {
            if (auto ctx = context_.lock()) {
                if (liveliness_callback_) {
                    liveliness_callback_(ctx, event);
                }
            }
        };

        // Create subscription once with all callbacks pre-registered
        subscription_ = context->node->template create_subscription<MessageT>(
            topic_name,
            qos,
            [this](typename MessageT::ConstSharedPtr msg) {
                auto ctx = context_.lock();
                if (!ctx)
                    return;
                if (ctx->node->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
                    callback_(ctx, msg);
                }
            },
            options
        );
    }

    void set_callback(std::function<void(std::shared_ptr<ContextType>, typename MessageT::ConstSharedPtr)> callback) {
        callback_ = callback;
    }

    void
    set_deadline_callback(std::function<void(std::shared_ptr<ContextType>, rclcpp::QOSDeadlineRequestedInfo &)> callback
    ) {
        deadline_callback_ = callback;
    }

    void set_liveliness_callback(
        std::function<void(std::shared_ptr<ContextType>, rclcpp::QOSLivelinessChangedInfo &)> callback
    ) {
        liveliness_callback_ = callback;
    }

    // Access underlying subscription for advanced use
    typename rclcpp::Subscription<MessageT>::SharedPtr subscription() { return subscription_; }

  private:
    std::weak_ptr<ContextType> context_;
    typename rclcpp::Subscription<MessageT>::SharedPtr subscription_;

    std::function<void(std::shared_ptr<ContextType>, typename MessageT::ConstSharedPtr)> callback_;
    std::function<void(std::shared_ptr<ContextType>, rclcpp::QOSDeadlineRequestedInfo &)> deadline_callback_;
    std::function<void(std::shared_ptr<ContextType>, rclcpp::QOSLivelinessChangedInfo &)> liveliness_callback_;
};

template <typename MessageT, typename ContextType>
std::shared_ptr<Subscriber<MessageT, ContextType>>
create_subscriber(std::shared_ptr<ContextType> context, const std::string &topic_name, const rclcpp::QoS &qos) {
    return std::make_shared<Subscriber<MessageT, ContextType>>(context, topic_name, qos);
}

} // namespace cake
