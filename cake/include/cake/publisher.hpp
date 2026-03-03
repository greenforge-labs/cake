#pragma once

#include <functional>
#include <memory>
#include <type_traits>

#include <rclcpp/rclcpp.hpp>

#include "session.hpp"

namespace cake {

template <typename MessageT, typename SessionType> class Publisher {
    static_assert(std::is_base_of_v<Session, SessionType>, "SessionType must derive from cake::Session");

  public:
    explicit Publisher(std::shared_ptr<SessionType> sn, const std::string &topic_name, const rclcpp::QoS &qos)
        : session_(sn) {
        rclcpp::PublisherOptions options;
        options.event_callbacks.deadline_callback = [this](rclcpp::QOSDeadlineOfferedInfo &event) {
            if (auto sn = session_.lock()) {
                if (deadline_callback_) {
                    deadline_callback_(sn, event);
                }
            }
        };
        options.event_callbacks.liveliness_callback = [this](rclcpp::QOSLivelinessLostInfo &event) {
            if (auto sn = session_.lock()) {
                if (liveliness_callback_) {
                    liveliness_callback_(sn, event);
                }
            }
        };

        publisher_ = rclcpp::create_publisher<MessageT>(sn->node, topic_name, qos, options);
    }

    void publish(const MessageT &msg) { publisher_->publish(msg); }
    void publish(std::unique_ptr<MessageT> msg) { publisher_->publish(std::move(msg)); }

    // Access underlying publisher for advanced use (wait_for_all_acked, get_subscription_count, etc.)
    typename rclcpp::Publisher<MessageT>::SharedPtr publisher() { return publisher_; }

    void
    set_deadline_callback(std::function<void(std::shared_ptr<SessionType>, rclcpp::QOSDeadlineOfferedInfo &)> callback
    ) {
        deadline_callback_ = callback;
    }

    void
    set_liveliness_callback(std::function<void(std::shared_ptr<SessionType>, rclcpp::QOSLivelinessLostInfo &)> callback
    ) {
        liveliness_callback_ = callback;
    }

  private:
    std::weak_ptr<SessionType> session_;
    typename rclcpp::Publisher<MessageT>::SharedPtr publisher_;

    std::function<void(std::shared_ptr<SessionType>, rclcpp::QOSDeadlineOfferedInfo &)> deadline_callback_;
    std::function<void(std::shared_ptr<SessionType>, rclcpp::QOSLivelinessLostInfo &)> liveliness_callback_;
};

template <typename MessageT, typename SessionType>
std::shared_ptr<Publisher<MessageT, SessionType>>
create_publisher(std::shared_ptr<SessionType> sn, const std::string &topic_name, const rclcpp::QoS &qos) {
    return std::make_shared<Publisher<MessageT, SessionType>>(sn, topic_name, qos);
}

} // namespace cake
