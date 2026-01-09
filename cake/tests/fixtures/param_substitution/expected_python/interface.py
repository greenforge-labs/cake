# auto-generated DO NOT EDIT

from __future__ import annotations

from dataclasses import dataclass, field

import rclpy
from rclpy.qos import (
    Duration,
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
    LivelinessPolicy,
)
from example_interfaces.action import Fibonacci
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from std_msgs.msg import String
from std_srvs.srv import Trigger

import cake

from typing import Callable, TypeVar

from .parameters import Params, ParamListener

# Mapping dictionaries for runtime enum substitution
_RELIABILITY_MAP = {"reliable": ReliabilityPolicy.RELIABLE, "best_effort": ReliabilityPolicy.BEST_EFFORT}
_DURABILITY_MAP = {"volatile": DurabilityPolicy.VOLATILE, "transient_local": DurabilityPolicy.TRANSIENT_LOCAL}
_HISTORY_MAP = {"keep_last": HistoryPolicy.KEEP_LAST, "keep_all": HistoryPolicy.KEEP_ALL}
_LIVELINESS_MAP = {"automatic": LivelinessPolicy.AUTOMATIC, "manual_by_topic": LivelinessPolicy.MANUAL_BY_TOPIC}


@dataclass
class Publishers:
    topic_prefix: cake.Publisher[String] = field(default_factory=cake.Publisher[String])
    topic_prefix_robot_name_status: cake.Publisher[String] = field(default_factory=cake.Publisher[String])
    cmd_vel: cake.Publisher[Twist] = field(default_factory=cake.Publisher[Twist])
    sensor_data: cake.Publisher[Float32] = field(default_factory=cake.Publisher[Float32])
    critical_data: cake.Publisher[String] = field(default_factory=cake.Publisher[String])


@dataclass
class Subscribers:
    topic_prefix_robot_name_command: cake.Subscriber[String] = field(default_factory=cake.Subscriber[String])


@dataclass
class Services:
    topic_prefix_robot_name_reset: cake.Service[Trigger, Trigger.Request, Trigger.Response] = field(default_factory=cake.Service[Trigger, Trigger.Request, Trigger.Response])


@dataclass
class ServiceClients:
    pass


@dataclass
class Actions:
    topic_prefix_robot_name_navigate: cake.SingleGoalActionServer[Fibonacci, Fibonacci.Goal, Fibonacci.Result, Fibonacci.Feedback]


@dataclass
class ActionClients:
    pass


@dataclass
class ParamSubstitutionContext(cake.Context):
    publishers: Publishers
    subscribers: Subscribers
    services: Services
    service_clients: ServiceClients
    actions: Actions
    action_clients: ActionClients

    param_listener: ParamListener
    params: Params


T = TypeVar("T", bound=ParamSubstitutionContext)


def run(context_type: type[T], init_func: Callable[[T], None]):

    rclpy.init()

    node = rclpy.create_node("param_substitution")

    # initialise parameters (before entities to support ${params.X} substitutions)
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
        topic_prefix_robot_name_navigate=cake.SingleGoalActionServer[Fibonacci, Fibonacci.Goal, Fibonacci.Result, Fibonacci.Feedback](node, Fibonacci, f"{params.topic_prefix}/{params.robot_name}/navigate"),
    )

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
    ctx.publishers.topic_prefix._initialise(ctx, String, params.topic_prefix, 10)
    ctx.publishers.topic_prefix_robot_name_status._initialise(ctx, String, f"{params.topic_prefix}/{params.robot_name}/status", params.qos_depth)
    ctx.publishers.cmd_vel._initialise(ctx, Twist, "/cmd_vel", QoSProfile(depth=params.qos_depth, reliability=ReliabilityPolicy.RELIABLE))
    ctx.publishers.sensor_data._initialise(ctx, Float32, "/sensor_data", QoSProfile(depth=params.qos_depth, reliability=_RELIABILITY_MAP[params.reliability_mode]))
    ctx.publishers.critical_data._initialise(ctx, String, "/critical_data", QoSProfile(depth=10, deadline=Duration(seconds=params.deadline_sec, nanoseconds=0)))

    # initialise subscribers
    ctx.subscribers.topic_prefix_robot_name_command._initialise(ctx, String, f"{params.topic_prefix}/{params.robot_name}/command", params.qos_depth)

    # initialise services
    ctx.services.topic_prefix_robot_name_reset._initialise(ctx, Trigger, f"{params.topic_prefix}/{params.robot_name}/reset")

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
