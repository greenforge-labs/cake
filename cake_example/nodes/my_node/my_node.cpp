#include "my_node.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include <memory>
#include <rclcpp/logging.hpp>

namespace cake_example::my_node {

void msg_callback(std::shared_ptr<Context> ctx, std_msgs::msg::Bool::ConstSharedPtr msg) {
    RCLCPP_INFO(
        ctx->node->get_logger(),
        "Got a bool: {%d}. BTW the very important number is: {%d}",
        msg->data,
        ctx->very_important_number
    );

    ctx->very_important_number++;
}

void init(std::shared_ptr<Context> ctx) {
    RCLCPP_INFO(ctx->node->get_logger(), "Hello from the test range! This is **my_node**.");

    auto msg = std_msgs::msg::String();
    msg.data = "fire!";
    ctx->publishers.some_topic->publish(msg);

    ctx->subscribers.other_topic->set_callback(msg_callback);
}

} // namespace cake_example::my_node

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(cake_example::my_node::MyNode);
