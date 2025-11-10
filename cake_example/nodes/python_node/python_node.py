from time import sleep

from cake_example.python_node.interface import PythonNodeContext, run
import rclpy

from std_msgs.msg import Bool, String

from example_interfaces.action import Fibonacci

import cake

from typing import cast


class Context(PythonNodeContext):
    important_number: float = 6.7
    count: int = 0


def topic_callback(ctx: Context, msg: Bool):
    response = f"Got message: {msg.data}"
    ctx.logger.info(response)
    ctx.publishers.a_topic.publish(String(data=response))


def thread_func(ctx: Context):
    while rclpy.ok():
        message = f"Hello from the thread! I can see and change context in here too: {ctx.important_number}"
        ctx.important_number += 1
        ctx.logger.info(message)
        ctx.publishers.a_topic.publish(String(data=message))
        sleep(1)


def action_func(ctx: Context):
    ctx.logger.info("Checking for action!")
    active_goal = ctx.actions.my_action_two.get_active_goal()
    if active_goal is None:
        ctx.count = 0

    if active_goal is not None:
        active_goal = cast(Fibonacci.Goal, active_goal)
        ctx.logger.info(f"Got goal: {active_goal.order}")
        ctx.count += 1

    if ctx.count > 10:
        result = Fibonacci.Result(sequence=[1, 2, 3, 4])
        ctx.actions.my_action_two.succeed(result)


def init(ctx: Context):
    ctx.logger.info("Hello from python cake!")
    ctx.logger.info(f"The parameter is: {ctx.params.special_number}. The context value is: {ctx.important_number}")
    ctx.subscribers.another_topic.set_callback(topic_callback)
    ctx.actions.my_action_two.set_options(cake.SingleGoalActionServerOptions(new_goals_replace_current_goal=True))
    cake.create_thread(ctx, thread_func)
    cake.create_timer(ctx, 1, action_func)


if __name__ == "__main__":
    run(Context, init)
