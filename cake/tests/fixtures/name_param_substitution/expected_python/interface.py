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
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool
from std_srvs.srv import Trigger

import cake

from typing import Callable, TypeVar

from .parameters import Params, ParamListener


@dataclass
class Publishers:
    cmd_vel: cake.Publisher[Twist] = field(default_factory=cake.Publisher[Twist])


@dataclass
class Subscribers:
    odom: cake.Subscriber[Odometry] = field(default_factory=cake.Subscriber[Odometry])


@dataclass
class Services:
    get_state: cake.Service[Trigger, Trigger.Request, Trigger.Response] = field(default_factory=cake.Service[Trigger, Trigger.Request, Trigger.Response])


@dataclass
class ServiceClients:
    external_service: Client  # srv_type: std_srvs/srv/SetBool


@dataclass
class Actions:
    pass


@dataclass
class ActionClients:
    pass


@dataclass
class NameParamSubstitutionSession(cake.Session):
    publishers: Publishers
    subscribers: Subscribers
    services: Services
    service_clients: ServiceClients
    actions: Actions
    action_clients: ActionClients

    param_listener: ParamListener
    params: Params


T = TypeVar("T", bound=NameParamSubstitutionSession)


class _NameParamSubstitutionNode(cake.BaseNode[T]):

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
            "name_param_substitution",
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
        service_clients = ServiceClients(
            external_service=node.create_client(SetBool, f"/{params.namespace}/service"),
        )

        # initialise actions
        actions = Actions()

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
        sn.publishers.cmd_vel._initialise(sn, Twist, f"/robot/{params.robot_id}/cmd_vel", QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10, reliability=ReliabilityPolicy.RELIABLE))

        # initialise subscribers
        sn.subscribers.odom._initialise(sn, Odometry, f"/{params.namespace}/{params.robot_id}/odom", QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=5, reliability=ReliabilityPolicy.BEST_EFFORT))

        # initialise services
        sn.services.get_state._initialise(sn, Trigger, f"/robot/{params.robot_id}/get_state")

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
        sn.publishers.cmd_vel._destroy(sn.node)
        sn.subscribers.odom._destroy(sn.node)
        sn.services.get_state._destroy(sn.node)
        sn.node.destroy_client(sn.service_clients.external_service)


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

    wrapper = _NameParamSubstitutionNode(
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
