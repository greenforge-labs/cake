#include "my_node.hpp"

#include <memory>
#include <rclcpp/logging.hpp>

#include "example_interfaces/srv/add_two_ints.hpp"
#include "example_interfaces/srv/trigger.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

#include <cake/timer.hpp>

using namespace std::chrono_literals;

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

void addition_request_handler(
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

void bing_bong_request_handler(
    std::shared_ptr<Context> ctx,
    example_interfaces::srv::Trigger::Request::SharedPtr /*request*/,
    example_interfaces::srv::Trigger::Response::SharedPtr response
) {
    response->success = true;
    response->message = "bing bong!";
    RCLCPP_INFO(ctx->node->get_logger(), "Bing bong requested!");
}

void init(std::shared_ptr<Context> ctx) {
    // Load parameters
    ctx->params = ParamListener(ctx->node).get_params();

    RCLCPP_INFO(ctx->node->get_logger(), "Hello from the test range! This is **my_node**.");
    RCLCPP_INFO(ctx->node->get_logger(), "spicy_param value: %s", ctx->params.spicy_param.c_str());

    auto msg = std_msgs::msg::String();
    msg.data = ctx->params.spicy_param;
    ctx->publishers.some_topic->publish(msg);

    ctx->subscribers.other_topic->set_callback(msg_callback);
    ctx->services.my_service->set_request_handler(addition_request_handler);
    ctx->services.bing_bong->set_request_handler(bing_bong_request_handler);

    cake::create_timer(ctx, 1000ms, [](auto ctx) {
        ctx->service_clients.bing_bong->async_send_request(std::make_shared<example_interfaces::srv::Trigger::Request>()
        );
    });
}

} // namespace cake_example::my_node

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(cake_example::my_node::MyNode);
