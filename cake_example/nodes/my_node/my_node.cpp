#include "my_node.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"
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

void request_handler(
    std::shared_ptr<Context> ctx,
    example_interfaces::srv::AddTwoInts::Request::SharedPtr request,
    example_interfaces::srv::AddTwoInts::Response::SharedPtr response
) {
    response->sum = request->a + request->b;
    RCLCPP_INFO(
        ctx->node->get_logger(),
        "Incoming request: a=%ld, b=%ld. Responding with sum=%ld",
        request->a,
        request->b,
        response->sum
    );
}

void init(std::shared_ptr<Context> ctx) {
    RCLCPP_INFO(ctx->node->get_logger(), "Hello from the test range! This is **my_node**.");

    auto msg = std_msgs::msg::String();
    msg.data = "fire!";
    ctx->publishers.some_topic->publish(msg);

    ctx->subscribers.other_topic->set_callback(msg_callback);
    ctx->services.my_service->set_request_handler(request_handler);
}

} // namespace cake_example::my_node

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(cake_example::my_node::MyNode);
