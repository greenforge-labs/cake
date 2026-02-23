# auto-generated DO NOT EDIT

from __future__ import annotations

from dataclasses import dataclass, field

import rclpy
from example_interfaces.action import Fibonacci

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
    fibonacci: cake.SingleGoalActionServer[Fibonacci, Fibonacci.Goal, Fibonacci.Result, Fibonacci.Feedback]
    math_compute: cake.SingleGoalActionServer[Fibonacci, Fibonacci.Goal, Fibonacci.Result, Fibonacci.Feedback]


@dataclass
class ActionClients:
    pass


@dataclass
class ActionServersOnlySession(cake.Session):
    publishers: Publishers
    subscribers: Subscribers
    services: Services
    service_clients: ServiceClients
    actions: Actions
    action_clients: ActionClients

    param_listener: ParamListener
    params: Params


T = TypeVar("T", bound=ActionServersOnlySession)


class _ActionServersOnlyNode(cake.BaseNode[T]):

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
            "action_servers_only",
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
        actions = Actions(
            fibonacci=cake.SingleGoalActionServer[Fibonacci, Fibonacci.Goal, Fibonacci.Result, Fibonacci.Feedback](node, Fibonacci, "fibonacci"),
            math_compute=cake.SingleGoalActionServer[Fibonacci, Fibonacci.Goal, Fibonacci.Result, Fibonacci.Feedback](node, Fibonacci, "/math/compute"),
        )

        # initialise action clients
        action_clients = ActionClients()

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
        sn.actions.fibonacci.deactivate()
        sn.actions.math_compute.deactivate()

    def _destroy_entities(self, sn: T) -> None:
        for timer in sn.timers:
            sn.node.destroy_timer(timer)
        sn.timers.clear()
        sn.actions.fibonacci._destroy(sn.node)
        sn.actions.math_compute._destroy(sn.node)


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

    wrapper = _ActionServersOnlyNode(
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
