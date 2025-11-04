from __future__ import annotations

from dataclasses import dataclass

import rclpy

import cake

from typing import Callable, TypeVar, TYPE_CHECKING

if TYPE_CHECKING:
    from cake.dummy_python_node import ParamListener, Params


class Publishers:
    pass


class Subscribers:
    pass


@dataclass
class PythonNodeContext(cake.Context):
    publishers: Publishers
    subscribers: Subscribers

    param_listener: ParamListener
    params: Params


T = TypeVar("T", bound=PythonNodeContext)


def run(context_type: type[T], init_func: Callable[[T], None]):

    rclpy.init()

    node = rclpy.create_node("PythonNode")

    # TODO: initialise publishers
    publishers = Publishers()

    # TODO: initialise subscribers
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

    init_func(ctx)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # note rclpy installs signal handlers on rclpy.init() that respond to SIGINT (Ctrl+C) and shutdown the context
        # so no logging or anything should be done here.
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            # since the context is _probably_ shutdown already here, we are doing this just to be certain
            rclpy.shutdown()
