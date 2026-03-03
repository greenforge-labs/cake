#pragma once

#include <memory>
#include <string>

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>

#include "subscriber.hpp"

namespace cake {

template <typename MessageT, typename SessionType>
void attach_default_qos_handlers(std::shared_ptr<Subscriber<MessageT, SessionType>> sub) {
    std::string topic = sub->subscription()->get_topic_name();

    sub->set_deadline_callback([topic](std::shared_ptr<SessionType> sn, rclcpp::QOSDeadlineRequestedInfo & /*event*/) {
        if (sn->node.get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
            return;
        }
        RCLCPP_ERROR(sn->node.get_logger(), "Subscriber '%s': deadline missed — deactivating node", topic.c_str());
        sn->node.deactivate();
    });

    sub->set_liveliness_callback([topic](std::shared_ptr<SessionType> sn, rclcpp::QOSLivelinessChangedInfo &event) {
        // Deactivate if there are no alive publishers remaining — covers both lease expiry and publisher removal.
        if (event.alive_count > 0) {
            return;
        }
        if (sn->node.get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
            return;
        }
        RCLCPP_ERROR(
            sn->node.get_logger(), "Subscriber '%s': topic has no alive publishers — deactivating node", topic.c_str()
        );
        sn->node.deactivate();
    });
}

} // namespace cake
