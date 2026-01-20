# auto-generated DO NOT EDIT

from __future__ import annotations

from dataclasses import dataclass, field

import rclpy
from rclpy.qos import (
    Duration,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

import cake
from cake.qos_helpers import _to_reliability, _to_durability, _to_liveliness

from typing import Callable, TypeVar

from .parameters import Params, ParamListener


@dataclass
class Publishers:
    processed_data: cake.Publisher[String] = field(default_factory=cake.Publisher[String])


@dataclass
class Subscribers:
    sensor_data: cake.Subscriber[LaserScan] = field(default_factory=cake.Subscriber[LaserScan])


@dataclass
class Services:
    pass


@dataclass
class ServiceClients:
    pass


@dataclass
class Actions:
    pass


@dataclass
class ActionClients:
    pass


@dataclass
class QosParamSubstitutionContext(cake.Context):
    publishers: Publishers
    subscribers: Subscribers
    services: Services
    service_clients: ServiceClients
    actions: Actions
    action_clients: ActionClients

    param_listener: ParamListener
    params: Params


T = TypeVar("T", bound=QosParamSubstitutionContext)


def run(context_type: type[T], init_func: Callable[[T], None]):

    rclpy.init()

    node = rclpy.create_node("qos_param_substitution")

    # create publishers - using default constructors
    publishers = Publishers()

    # create subscribers - using default constructors
    subscribers = Subscribers()

    # create services - using default constructors
    services = Services()

    # initialise service clients
    service_clients = ServiceClients()

    # initialise actions
    actions = Actions()

    # initialise action clients
    action_clients = ActionClients()

    param_listener = ParamListener(node)
    params = param_listener.get_params()

    ctx = context_type(
        node=node,
        publishers=publishers,
        subscribers=subscribers,
        services=services,
        service_clients=service_clients,
        actions=actions,
        action_clients=action_clients,
        param_listener=param_listener,
        params=params,
    )

    # initialise publishers
    ctx.publishers.processed_data._initialise(ctx, String, "/processed_data", QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=ctx.params.output_queue_depth, reliability=ReliabilityPolicy.RELIABLE))

    # initialise subscribers
    ctx.subscribers.sensor_data._initialise(ctx, LaserScan, "/sensor_data", QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=ctx.params.sensor_queue_depth, reliability=_to_reliability(ctx.params.sensor_reliability), durability=_to_durability(ctx.params.sensor_durability), deadline=Duration(nanoseconds=ctx.params.sensor_deadline_ms * 1000000), liveliness=_to_liveliness(ctx.params.sensor_liveliness)))

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
