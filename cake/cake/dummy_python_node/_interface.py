from __future__ import annotations

from dataclasses import dataclass, field

import rclpy
from rclpy.publisher import Publisher
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default

from std_msgs.msg import Bool, String

import cake

from typing import TYPE_CHECKING, Callable, TypeVar

if TYPE_CHECKING:
    from cake.dummy_python_node import ParamListener, Params


@dataclass
class Publishers:
    a_topic: Publisher  # msg_type: std_msgs/msg/String


@dataclass
class Subscribers:
    another_topic: cake.Subscriber[Bool] = field(default_factory=cake.Subscriber[Bool])


@dataclass
class PythonNodeContext(cake.Context):
    publishers: Publishers
    subscribers: Subscribers

    param_listener: ParamListener
    params: Params


T = TypeVar("T", bound=PythonNodeContext)


def run(context_type: type[T], init_func: Callable[[T], None]):

    rclpy.init()

    node = rclpy.create_node("python_node")

    # initialise publishers
    publishers = Publishers(
        a_topic=node.create_publisher(String, "a_topic", qos_profile_system_default),
    )

    # create subscribers - using default constructors
    subscribers = Subscribers()

    from cake.dummy_python_node import ParamListener

    param_listener = ParamListener(node)
    params = param_listener.get_params()

    ctx = context_type(
        node=node,
        publishers=publishers,
        subscribers=subscribers,
        param_listener=param_listener,
        params=params,
    )

    # initialise subscribers
    ctx.subscribers.another_topic._initialise(ctx, Bool, "another_topic", qos_profile_sensor_data)

    init_func(ctx)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # note rclpy installs signal handlers during rclpy.init() that respond to SIGINT (Ctrl+C) and shutdown the
        # context so no logging or anything should be done here.
        pass
    finally:
        for thread in ctx.threads:
            thread.join(timeout=2.0)  # give each thread 2 seconds to finish gracefully
        node.destroy_node()
        if rclpy.ok():
            # since the context is _probably_ shutdown already here, we are doing this just to be certain
            rclpy.shutdown()
