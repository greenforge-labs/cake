#pragma once

#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>

namespace cake {

template <typename MessageT, typename ContextType> class Subscriber {
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
            if (deadline_callback_) {
                deadline_callback_(context_, event);
            }
        };
        options.event_callbacks.liveliness_callback = [this](rclcpp::QOSLivelinessChangedInfo &event) {
            if (liveliness_callback_) {
                liveliness_callback_(context_, event);
            }
        };

        // Create subscription once with all callbacks pre-registered
        subscription_ = context->node->template create_subscription<MessageT>(
            topic_name, qos, [this](typename MessageT::ConstSharedPtr msg) { callback_(context_, msg); }, options
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

  private:
    std::shared_ptr<ContextType> context_;
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
