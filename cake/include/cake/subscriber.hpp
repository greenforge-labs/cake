#pragma once

#include <functional>

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

namespace cake {

template <typename MessageT, typename ContextType> class Subscriber {
  public:
    explicit Subscriber(std::shared_ptr<ContextType> context, const std::string &topic_name, const rclcpp::QoS &qos) {
        subscription_ = context->node->template create_subscription<MessageT>(
            topic_name, qos, [context, this](typename MessageT::ConstSharedPtr msg) { callback_(context, msg); }
        );
    };

    void set_callback(std::function<void(std::shared_ptr<ContextType>, typename MessageT::ConstSharedPtr)> callback) {
        callback_ = callback;
    }

  private:
    rclcpp::Subscription<MessageT>::SharedPtr subscription_;
    std::function<void(std::shared_ptr<ContextType>, typename MessageT::ConstSharedPtr)> callback_ = [](auto ctx,
                                                                                                        auto /*msg*/) {
        RCLCPP_WARN(
            ctx->node->get_logger(), "Subscriber received message but no callback configured. Call set_callback()."
        );
    };
};

template <typename MessageT, typename ContextType>
std::shared_ptr<Subscriber<MessageT, ContextType>>
create_subscriber(std::shared_ptr<ContextType> context, const std::string &topic_name, const rclcpp::QoS &qos) {
    return std::make_shared<Subscriber<MessageT, ContextType>>(context, topic_name, qos);
};
} // namespace cake
