# auto-generated DO NOT EDIT

from __future__ import annotations

from dataclasses import dataclass, field

import rclpy
from rclpy.publisher import Publisher
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import cake

from typing import Callable, TypeVar

from .parameters import Params, ParamListener


@dataclass
class Publishers:
    cmd_vel: Publisher  # msg_type: geometry_msgs/msg/Twist


@dataclass
class Subscribers:
    odom: cake.Subscriber[Odometry] = field(default_factory=cake.Subscriber[Odometry])


@dataclass
class Services:
    pass


@dataclass
class ServiceClients:
    pass


@dataclass
class SimpleNodeContext(cake.Context):
    publishers: Publishers
    subscribers: Subscribers
    services: Services
    service_clients: ServiceClients

    param_listener: ParamListener
    params: Params


T = TypeVar("T", bound=SimpleNodeContext)


def run(context_type: type[T], init_func: Callable[[T], None]):

    rclpy.init()

    node = rclpy.create_node("simple_node")

    # initialise publishers
    publishers = Publishers(
        cmd_vel=node.create_publisher(Twist, "/cmd_vel", 10),
    )

    # create subscribers - using default constructors
    subscribers = Subscribers()

    # create services - using default constructors
    services = Services()

    # initialise service clients
    service_clients = ServiceClients()

    param_listener = ParamListener(node)
    params = param_listener.get_params()

    ctx = context_type(
        node=node,
        publishers=publishers,
        subscribers=subscribers,
        services=services,
        service_clients=service_clients,
        param_listener=param_listener,
        params=params,
    )

    # initialise subscribers
    ctx.subscribers.odom._initialise(ctx, Odometry, "/odom", 10)

    # initialise services

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
