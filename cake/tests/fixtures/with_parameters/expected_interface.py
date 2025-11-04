# auto-generated DO NOT EDIT

from __future__ import annotations

from dataclasses import dataclass, field

import rclpy
from rclpy.publisher import Publisher

from std_msgs.msg import String

import cake

from typing import TYPE_CHECKING, Callable, TypeVar

if TYPE_CHECKING:
    from test_package.test_node._parameters import parameters


@dataclass
class Publishers:
    status: Publisher  # msg_type: std_msgs/msg/String


@dataclass
class Subscribers:
    pass


@dataclass
class TestNodeContext(cake.Context):
    publishers: Publishers
    subscribers: Subscribers

    param_listener: parameters.ParamListener
    params: parameters.Params


T = TypeVar("T", bound=TestNodeContext)


def run(context_type: type[T], init_func: Callable[[T], None]):

    rclpy.init()

    node = rclpy.create_node("test_node")

    # initialise publishers
    publishers = Publishers(
        status=node.create_publisher(String, "/status", 10),
    )

    # create subscribers - using default constructors
    subscribers = Subscribers()

    from test_package.test_node._parameters import parameters

    param_listener = parameters.ParamListener(node)
    params = param_listener.get_params()

    ctx = context_type(
        node=node,
        publishers=publishers,
        subscribers=subscribers,
        param_listener=param_listener,
        params=params,
    )

    # initialise subscribers

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
