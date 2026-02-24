from rclpy.callback_groups import CallbackGroup
from rclpy.clock import Clock
from rclpy.clock_type import ClockType

from lifecycle_msgs.msg import State

from .session import Session

from typing import Callable, TypeVar

SessionT = TypeVar("SessionT", bound=Session)


def create_timer(
    session: SessionT,
    timer_period_sec: float,
    callback: Callable[[SessionT], None],
    callback_group: CallbackGroup | None = None,
    clock: Clock | None = None,
    autostart: bool = False,
):
    def guarded_callback():
        if session.node.current_state != State.PRIMARY_STATE_ACTIVE:
            return
        callback(session)

    timer = session.node.create_timer(
        timer_period_sec,
        callback=guarded_callback,
        callback_group=callback_group,
        clock=clock,
        autostart=autostart,
    )
    session.timers.append(timer)


def create_wall_timer(
    session: SessionT,
    timer_period_sec: float,
    callback: Callable[[SessionT], None],
    callback_group: CallbackGroup | None = None,
    autostart: bool = False,
):
    def guarded_callback():
        if session.node.current_state != State.PRIMARY_STATE_ACTIVE:
            return
        callback(session)

    timer = session.node.create_timer(
        timer_period_sec,
        callback=guarded_callback,
        callback_group=callback_group,
        clock=Clock(clock_type=ClockType.STEADY_TIME),
        autostart=autostart,
    )
    session.timers.append(timer)


__all__ = ["create_timer", "create_wall_timer"]
