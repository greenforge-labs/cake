#include "my_node.hpp"

#include <memory>
#include <rclcpp/logging.hpp>

#include "example_interfaces/action/fibonacci.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"
#include "example_interfaces/srv/trigger.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

#include <cake/call_sync.hpp>
#include <cake/timer.hpp>

using namespace std::chrono_literals;

namespace cake_example::my_node {

void msg_callback(std::shared_ptr<Session> sn, std_msgs::msg::Bool::ConstSharedPtr msg) {
    RCLCPP_INFO(
        sn->node.get_logger(),
        "Got a bool: {%d}. BTW the very important number is: {%d}",
        msg->data,
        sn->very_important_number
    );

    sn->very_important_number++;
}

void addition_request_handler(
    std::shared_ptr<Session> sn,
    example_interfaces::srv::AddTwoInts::Request::SharedPtr request,
    example_interfaces::srv::AddTwoInts::Response::SharedPtr response
) {
    response->sum = request->a + request->b;
    RCLCPP_INFO(
        sn->node.get_logger(),
        "Incoming request: a=%ld, b=%ld. Responding with sum=%ld",
        request->a,
        request->b,
        response->sum
    );
}

void bing_bong_request_handler(
    std::shared_ptr<Session> sn,
    example_interfaces::srv::Trigger::Request::SharedPtr /*request*/,
    example_interfaces::srv::Trigger::Response::SharedPtr response
) {
    response->success = true;
    response->message = "bing bong!";
    RCLCPP_INFO(sn->node.get_logger(), "Bing bong requested!");
}

void action_callback(std::shared_ptr<Session> sn) {
    auto active_goal = sn->actions.my_action->get_active_goal();
    if (!active_goal) {
        sn->count = 0;
    }

    if (active_goal) {
        RCLCPP_INFO(sn->node.get_logger(), "Got goal! Order: %d", active_goal->order);
        sn->count++;
    }

    if (sn->count > 10) {
        auto result = std::make_shared<example_interfaces::action::Fibonacci::Result>();
        result->sequence.push_back(1);
        result->sequence.push_back(2);
        result->sequence.push_back(3);
        result->sequence.push_back(4);
        sn->actions.my_action->succeed(result);
    }
}

CallbackReturn on_configure(std::shared_ptr<Session> sn) {
    RCLCPP_INFO(sn->node.get_logger(), "Hello from the test range! This is **my_node**.");
    RCLCPP_INFO(sn->node.get_logger(), "spicy_param value: %s", sn->params.spicy_param.c_str());

    auto msg = std_msgs::msg::String();
    msg.data = sn->params.spicy_param;
    sn->publishers.some_topic->publish(msg);

    sn->subscribers.other_topic->set_callback(msg_callback);
    sn->services.my_service->set_request_handler(addition_request_handler);
    sn->services.bing_bong->set_request_handler(bing_bong_request_handler);

    cake::create_timer(sn, 1000ms, [](std::shared_ptr<Session> sn) {
        sn->service_clients.bing_bong->async_send_request(std::make_shared<example_interfaces::srv::Trigger::Request>()
        );
    });

    sn->actions.my_action->set_options({.new_goals_replace_current_goal = true}); // accept defaults
    cake::create_timer(sn, 1000ms, action_callback);

    // Publish heartbeat every 200ms — python_node subscribes with a 1s deadline,
    // so if this node deactivates the subscriber will miss its deadline and also deactivate.
    cake::create_timer(sn, 200ms, [](std::shared_ptr<Session> sn) {
        auto msg = std_msgs::msg::Bool();
        msg.data = true;
        sn->publishers.heartbeat->publish(msg);
    });

    // Sync service call — calls python_node's my_other_service from on_configure without
    // deadlocking, because cake puts service client responses on an isolated background executor.
    if (sn->service_clients.my_other_service->wait_for_service(2s)) {
        auto req = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
        req->a = 3;
        req->b = 4;
        auto resp = cake::call_sync<example_interfaces::srv::AddTwoInts>(sn->service_clients.my_other_service, req, 5s);
        if (resp) {
            RCLCPP_INFO(sn->node.get_logger(), "Sync service call: 3 + 4 = %ld", resp->sum);
        } else {
            RCLCPP_WARN(sn->node.get_logger(), "Sync service call timed out");
        }
    } else {
        RCLCPP_INFO(sn->node.get_logger(), "my_other_service not available yet, skipping sync call");
    }

    // Sync action goal — calls python_node's my_action_two. send_goal_sync blocks until the
    // goal is accepted (or times out), which is safe from on_configure for the same reason.
    if (sn->action_clients.my_action_two->wait_for_action_server(2s)) {
        example_interfaces::action::Fibonacci::Goal goal;
        goal.order = 5;
        auto goal_handle =
            cake::send_goal_sync<example_interfaces::action::Fibonacci>(sn->action_clients.my_action_two, goal, {}, 5s);
        if (goal_handle) {
            RCLCPP_INFO(sn->node.get_logger(), "Sync action goal accepted");
        } else {
            RCLCPP_WARN(sn->node.get_logger(), "Sync action goal rejected or timed out");
        }
    } else {
        RCLCPP_INFO(sn->node.get_logger(), "my_action_two not available yet, skipping sync goal");
    }

    return CallbackReturn::SUCCESS;
}

} // namespace cake_example::my_node
