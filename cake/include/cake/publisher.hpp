#pragma once

#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>

namespace cake {

template <typename MessageT, typename ContextType> class Publisher {
  public:
    explicit Publisher(std::shared_ptr<ContextType> context, const std::string &topic_name, const rclcpp::QoS &qos)
        : context_(context) {
        rclcpp::PublisherOptions options;
        options.event_callbacks.deadline_callback = [this](rclcpp::QOSDeadlineOfferedInfo &event) {
            if (deadline_callback_) {
                deadline_callback_(context_, event);
            }
        };
        options.event_callbacks.liveliness_callback = [this](rclcpp::QOSLivelinessLostInfo &event) {
            if (liveliness_callback_) {
                liveliness_callback_(context_, event);
            }
        };

        publisher_ = context->node->template create_publisher<MessageT>(topic_name, qos, options);
    }

    void publish(const MessageT &msg) { publisher_->publish(msg); }
    void publish(std::unique_ptr<MessageT> msg) { publisher_->publish(std::move(msg)); }

    // Access underlying publisher for advanced use (wait_for_all_acked, get_subscription_count, etc.)
    typename rclcpp::Publisher<MessageT>::SharedPtr publisher() { return publisher_; }

    void
    set_deadline_callback(std::function<void(std::shared_ptr<ContextType>, rclcpp::QOSDeadlineOfferedInfo &)> callback
    ) {
        deadline_callback_ = callback;
    }

    void
    set_liveliness_callback(std::function<void(std::shared_ptr<ContextType>, rclcpp::QOSLivelinessLostInfo &)> callback
    ) {
        liveliness_callback_ = callback;
    }

  private:
    std::shared_ptr<ContextType> context_;
    typename rclcpp::Publisher<MessageT>::SharedPtr publisher_;

    std::function<void(std::shared_ptr<ContextType>, rclcpp::QOSDeadlineOfferedInfo &)> deadline_callback_;
    std::function<void(std::shared_ptr<ContextType>, rclcpp::QOSLivelinessLostInfo &)> liveliness_callback_;
};

template <typename MessageT, typename ContextType>
std::shared_ptr<Publisher<MessageT, ContextType>>
create_publisher(std::shared_ptr<ContextType> context, const std::string &topic_name, const rclcpp::QoS &qos) {
    return std::make_shared<Publisher<MessageT, ContextType>>(context, topic_name, qos);
}

} // namespace cake
