# auto-generated DO NOT EDIT

from __future__ import annotations

from dataclasses import dataclass, field

import rclpy
from rclpy.qos import (
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String

import cake

from typing import Callable, TypeVar

from .parameters import Params, ParamListener


@dataclass
class Publishers:
    sensor_data_topic: cake.Publisher[String] = field(default_factory=cake.Publisher[String])
    system_defaults_topic: cake.Publisher[String] = field(default_factory=cake.Publisher[String])


@dataclass
class Subscribers:
    parameters_topic: cake.Subscriber[String] = field(default_factory=cake.Subscriber[String])
    services_topic: cake.Subscriber[String] = field(default_factory=cake.Subscriber[String])


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
class QosPredefinedContext(cake.Context):
    publishers: Publishers
    subscribers: Subscribers
    services: Services
    service_clients: ServiceClients
    actions: Actions
    action_clients: ActionClients

    param_listener: ParamListener
    params: Params


T = TypeVar("T", bound=QosPredefinedContext)


def run(context_type: type[T], init_func: Callable[[T], None]):

    rclpy.init()

    node = rclpy.create_node("qos_predefined")

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
    ctx.publishers.sensor_data_topic._initialise(ctx, String, "sensor_data_topic", QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=5, reliability=ReliabilityPolicy.BEST_EFFORT))
    ctx.publishers.system_defaults_topic._initialise(ctx, String, "system_defaults_topic", QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10, reliability=ReliabilityPolicy.RELIABLE))

    # initialise subscribers
    ctx.subscribers.parameters_topic._initialise(ctx, String, "parameters_topic", QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10, reliability=ReliabilityPolicy.RELIABLE))
    ctx.subscribers.services_topic._initialise(ctx, String, "services_topic", QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10, reliability=ReliabilityPolicy.RELIABLE))

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
