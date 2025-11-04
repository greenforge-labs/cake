from time import sleep

import rclpy

from std_msgs.msg import Bool, String

import cake

# from cake_example.python_node import PythonNodeContext, run
from cake.dummy_python_node import PythonNodeContext, run


class Context(PythonNodeContext):
    important_number: float = 6.7


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


def init(ctx: Context):
    ctx.logger.info("Hello from python cake!")
    ctx.logger.info(f"The parameter is: {ctx.params.special_number}. The context value is: {ctx.important_number}")
    ctx.subscribers.another_topic.set_callback(topic_callback)
    cake.create_thread(ctx, thread_func)


if __name__ == "__main__":
    run(Context, init)
