from rclpy.callback_groups import CallbackGroup
from rclpy.clock import Clock

from .context import Context

from typing import Callable, TypeVar

ContextT = TypeVar("ContextT", bound=Context)


def create_timer(
    context: ContextT,
    timer_period_sec: float,
    callback: Callable[[ContextT], None],
    callback_group: CallbackGroup | None = None,
    clock: Clock | None = None,
    autostart: bool = True,
):
    timer = context.node.create_timer(
        timer_period_sec,
        callback=lambda: callback(context),
        callback_group=callback_group,
        clock=clock,
        autostart=autostart,
    )
    context.timers.append(timer)


__all__ = ["create_timer"]
