#pragma once

#include <functional>
#include <memory>
#include <type_traits>

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>

#include "session.hpp"

namespace cake {

template <typename MessageT, typename SessionType> class Subscriber {
    static_assert(std::is_base_of_v<Session, SessionType>, "SessionType must derive from cake::Session");

  public:
    explicit Subscriber(std::shared_ptr<SessionType> sn, const std::string &topic_name, const rclcpp::QoS &qos)
        : session_(sn) {
        set_callback([topic_name](auto sn, auto /*msg*/) {
            RCLCPP_WARN(
                sn->node.get_logger(),
                "Subscriber '%s' received message but no callback configured. Call set_callback().",
                topic_name.c_str()
            );
        });

        // Register noop callbacks that delegate to optional user callbacks
        rclcpp::SubscriptionOptions options;
        options.event_callbacks.deadline_callback = [this](rclcpp::QOSDeadlineRequestedInfo &event) {
            if (auto sn = session_.lock()) {
                if (deadline_callback_) {
                    deadline_callback_(sn, event);
                }
            }
        };
        options.event_callbacks.liveliness_callback = [this](rclcpp::QOSLivelinessChangedInfo &event) {
            if (auto sn = session_.lock()) {
                if (liveliness_callback_) {
                    liveliness_callback_(sn, event);
                }
            }
        };

        // Create subscription once with all callbacks pre-registered
        subscription_ = sn->node.template create_subscription<MessageT>(
            topic_name,
            qos,
            [this](typename MessageT::ConstSharedPtr msg) {
                auto sn = session_.lock();
                if (!sn)
                    return;
                if (sn->node.get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
                    callback_(sn, msg);
                }
            },
            options
        );
    }

    void set_callback(std::function<void(std::shared_ptr<SessionType>, typename MessageT::ConstSharedPtr)> callback) {
        callback_ = callback;
    }

    void
    set_deadline_callback(std::function<void(std::shared_ptr<SessionType>, rclcpp::QOSDeadlineRequestedInfo &)> callback
    ) {
        deadline_callback_ = callback;
    }

    void set_liveliness_callback(
        std::function<void(std::shared_ptr<SessionType>, rclcpp::QOSLivelinessChangedInfo &)> callback
    ) {
        liveliness_callback_ = callback;
    }

    // Access underlying subscription for advanced use
    typename rclcpp::Subscription<MessageT>::SharedPtr subscription() { return subscription_; }

  private:
    std::weak_ptr<SessionType> session_;
    typename rclcpp::Subscription<MessageT>::SharedPtr subscription_;

    std::function<void(std::shared_ptr<SessionType>, typename MessageT::ConstSharedPtr)> callback_;
    std::function<void(std::shared_ptr<SessionType>, rclcpp::QOSDeadlineRequestedInfo &)> deadline_callback_;
    std::function<void(std::shared_ptr<SessionType>, rclcpp::QOSLivelinessChangedInfo &)> liveliness_callback_;
};

template <typename MessageT, typename SessionType>
std::shared_ptr<Subscriber<MessageT, SessionType>>
create_subscriber(std::shared_ptr<SessionType> sn, const std::string &topic_name, const rclcpp::QoS &qos) {
    return std::make_shared<Subscriber<MessageT, SessionType>>(sn, topic_name, qos);
}

} // namespace cake
