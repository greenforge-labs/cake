# auto-generated DO NOT EDIT

from __future__ import annotations

from dataclasses import dataclass, field

import rclpy
from rclpy.client import Client
from rclpy.qos import (
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from lifecycle_msgs.srv import ChangeState
from std_msgs.msg import String

import cake

from typing import Callable, TypeVar

from .parameters import Params, ParamListener


@dataclass
class Publishers:
    status: cake.Publisher[String] = field(default_factory=cake.Publisher[String])


@dataclass
class Subscribers:
    node_states: dict[str, cake.Subscriber[String]] = field(default_factory=dict)


@dataclass
class Services:
    pass


@dataclass
class ServiceClients:
    change_state_clients: dict[str, Client] = field(default_factory=dict)  # srv_type: lifecycle_msgs/srv/ChangeState


@dataclass
class Actions:
    pass


@dataclass
class ActionClients:
    pass


@dataclass
class ForEachParamContext(cake.Context):
    publishers: Publishers
    subscribers: Subscribers
    services: Services
    service_clients: ServiceClients
    actions: Actions
    action_clients: ActionClients

    param_listener: ParamListener
    params: Params


T = TypeVar("T", bound=ForEachParamContext)


def run(context_type: type[T], init_func: Callable[[T], None]):

    rclpy.init()

    node = rclpy.create_node("for_each_param")

    # init parameters (must be before publishers/subscribers for param refs in names)
    param_listener = ParamListener(node)
    params = param_listener.get_params()

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
    ctx.publishers.status._initialise(ctx, String, f"/robot/{params.robot_id}/status", QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10, reliability=ReliabilityPolicy.RELIABLE))

    # initialise subscribers
    for key in params.managed_nodes:
        ctx.subscribers.node_states[key] = cake.Subscriber[String]()
        ctx.subscribers.node_states[key]._initialise(ctx, String, f"/{key}/state", QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10, reliability=ReliabilityPolicy.RELIABLE))

    # initialise services

    # initialise for_each_param service clients: change_state_clients
    for key in params.managed_nodes:
        ctx.service_clients.change_state_clients[key] = node.create_client(ChangeState, f"/{key}/change_state")

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
