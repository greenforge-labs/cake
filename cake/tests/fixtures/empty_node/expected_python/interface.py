# auto-generated DO NOT EDIT

from __future__ import annotations

from dataclasses import dataclass, field

import rclpy

import cake

from typing import Callable, TypeVar

from .parameters import Params, ParamListener


@dataclass
class Publishers:
    pass


@dataclass
class Subscribers:
    pass


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
class EmptyNodeContext(cake.Context):
    publishers: Publishers
    subscribers: Subscribers
    services: Services
    service_clients: ServiceClients
    actions: Actions
    action_clients: ActionClients

    param_listener: ParamListener
    params: Params


T = TypeVar("T", bound=EmptyNodeContext)


def run(context_type: type[T], init_func: Callable[[T], None]):

    rclpy.init()

    node = rclpy.create_node("empty_node")

    # initialise publishers
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

    # initialise subscribers

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
