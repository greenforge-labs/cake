from rclpy.qos import QoSProfile
from rclpy.subscription import Subscription

from .context import Context

from typing import Any, Callable, Generic, TypeVar, cast

MessageT = TypeVar("MessageT")


def get_no_callback_warning_logger(topic_name: str) -> Callable[[Context, Any], None]:
    def inner(ctx: Context, msg: Any):
        ctx.node.get_logger().warning(
            f"Subscriber {topic_name} received message but no callback configured. Call set_callback()."
        )

    return inner


class Subscriber(Generic[MessageT]):
    _subscription: Subscription | None = None
    _callback: Callable[[Any, MessageT], None]

    def _initialise(
        self,
        context: Context,
        msg_type: type[MessageT],
        topic_name: str,
        qos: QoSProfile | int,
    ) -> None:
        self._callback = get_no_callback_warning_logger(topic_name)
        self._subscription = context.node.create_subscription(
            msg_type=msg_type,
            topic=topic_name,
            callback=lambda msg: self._callback(context, cast(MessageT, msg)),
            qos_profile=qos,
        )

    def set_callback(self, callback: Callable[[Any, MessageT], None]):
        if self._subscription is None:
            raise RuntimeError("Can't set callback. Subscriber has not been initialised! This is an error in cake.")
        self._callback = callback


__all__ = ["Subscriber"]
