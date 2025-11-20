# auto-generated DO NOT EDIT

from __future__ import annotations

from dataclasses import dataclass, field

import rclpy
from rclpy.publisher import Publisher
from rclpy.qos import (
    qos_profile_sensor_data,
    qos_profile_services_default,
    QoSProfile,
    ReliabilityPolicy,
)
from example_interfaces.srv import AddTwoInts
from std_msgs.msg import String
from std_srvs.srv import Trigger

import cake

from typing import Callable, TypeVar

from .parameters import Params, ParamListener


@dataclass
class Publishers:
    status: Publisher  # msg_type: std_msgs/msg/String


@dataclass
class Subscribers:
    command: cake.Subscriber[String] = field(default_factory=cake.Subscriber[String])


@dataclass
class Services:
    reset: cake.Service[Trigger, Trigger.Request, Trigger.Response] = field(default_factory=cake.Service[Trigger, Trigger.Request, Trigger.Response])
    compute: cake.Service[AddTwoInts, AddTwoInts.Request, AddTwoInts.Response] = field(default_factory=cake.Service[AddTwoInts, AddTwoInts.Request, AddTwoInts.Response])


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
class ServicesWithPubsubContext(cake.Context):
    publishers: Publishers
    subscribers: Subscribers
    services: Services
    service_clients: ServiceClients
    actions: Actions
    action_clients: ActionClients

    param_listener: ParamListener
    params: Params


T = TypeVar("T", bound=ServicesWithPubsubContext)


def run(context_type: type[T], init_func: Callable[[T], None]):

    rclpy.init()

    node = rclpy.create_node("services_with_pubsub")

    # initialise publishers
    publishers = Publishers(
        status=node.create_publisher(String, "/status", 10),
    )

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

    # initialise subscribers
    ctx.subscribers.command._initialise(ctx, String, "/command", qos_profile_sensor_data)

    # initialise services
    ctx.services.reset._initialise(ctx, Trigger, "/reset", qos_profile_services_default)
    ctx.services.compute._initialise(ctx, AddTwoInts, "compute", QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE))

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
