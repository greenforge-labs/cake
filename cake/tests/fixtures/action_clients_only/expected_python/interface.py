# auto-generated DO NOT EDIT

from __future__ import annotations

from dataclasses import dataclass, field

import rclpy
from rclpy.action import ActionClient
from example_interfaces.action import Fibonacci
from nav2_msgs.action import NavigateToPose

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
    fibonacci: ActionClient  # action_type: example_interfaces/action/Fibonacci
    navigate_to_pose: ActionClient  # action_type: nav2_msgs/action/NavigateToPose


@dataclass
class ActionClientsOnlySession(cake.Session):
    publishers: Publishers
    subscribers: Subscribers
    services: Services
    service_clients: ServiceClients
    actions: Actions
    action_clients: ActionClients

    param_listener: ParamListener
    params: Params


T = TypeVar("T", bound=ActionClientsOnlySession)


class _ActionClientsOnlyNode(cake.BaseNode[T]):

    def __init__(
        self,
        session_type: type[T],
        on_configure: Callable[[T], cake.TransitionCallbackReturn],
        *,
        on_activate: Callable[[T], cake.TransitionCallbackReturn] | None = None,
        on_deactivate: Callable[[T], cake.TransitionCallbackReturn] | None = None,
        on_cleanup: Callable[[T], cake.TransitionCallbackReturn] | None = None,
        on_shutdown: Callable[[T], None] | None = None,
    ) -> None:
        super().__init__(
            "action_clients_only",
            session_type,
            on_configure,
            on_activate=on_activate,
            on_deactivate=on_deactivate,
            on_cleanup=on_cleanup,
            on_shutdown=on_shutdown,
        )

    def _create_session(self, node) -> T:
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
        action_clients = ActionClients(
            fibonacci=ActionClient(node, Fibonacci, "/fibonacci"),
            navigate_to_pose=ActionClient(node, NavigateToPose, "navigate_to_pose"),
        )

        sn = self._session_type(
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

        # initialise subscribers

        # initialise services

        return sn

    def _activate_entities(self, sn: T) -> None:
        for timer in sn.timers:
            timer.reset()

    def _deactivate_entities(self, sn: T) -> None:
        for timer in sn.timers:
            timer.cancel()

    def _destroy_entities(self, sn: T) -> None:
        for timer in sn.timers:
            sn.node.destroy_timer(timer)
        sn.timers.clear()
        sn.action_clients.fibonacci.destroy()
        sn.action_clients.navigate_to_pose.destroy()


def run(
    session_type: type[T],
    on_configure: Callable[[T], cake.TransitionCallbackReturn],
    *,
    on_activate: Callable[[T], cake.TransitionCallbackReturn] | None = None,
    on_deactivate: Callable[[T], cake.TransitionCallbackReturn] | None = None,
    on_cleanup: Callable[[T], cake.TransitionCallbackReturn] | None = None,
    on_shutdown: Callable[[T], None] | None = None,
):

    rclpy.init()

    wrapper = _ActionClientsOnlyNode(
        session_type,
        on_configure,
        on_activate=on_activate,
        on_deactivate=on_deactivate,
        on_cleanup=on_cleanup,
        on_shutdown=on_shutdown,
    )

    try:
        rclpy.spin(wrapper.node)
    except KeyboardInterrupt:
        # note rclpy installs signal handlers during rclpy.init() that respond to SIGINT (Ctrl+C) and shutdown the
        # context so no logging or anything should be done here.
        pass
    finally:
        wrapper.node.destroy_node()
        if rclpy.ok():
            # since the context is _probably_ shutdown already here, we are doing this just to be certain
            rclpy.shutdown()
