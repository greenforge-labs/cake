from rclpy.event_handler import PublisherEventCallbacks, QoSLivelinessLostInfo, QoSOfferedDeadlineMissedInfo
from rclpy.publisher import Publisher as RclpyPublisher
from rclpy.qos import QoSProfile

from .context import Context

from typing import Any, Callable, Generic, Optional, TypeVar

MessageT = TypeVar("MessageT")


class Publisher(Generic[MessageT]):
    _publisher: RclpyPublisher | None = None
    _deadline_callback: Optional[Callable[[Any, QoSOfferedDeadlineMissedInfo], None]] = None
    _liveliness_callback: Optional[Callable[[Any, QoSLivelinessLostInfo], None]] = None

    def _initialise(
        self,
        context: Context,
        msg_type: type[MessageT],
        topic_name: str,
        qos: QoSProfile | int,
    ) -> None:
        event_callbacks = PublisherEventCallbacks(
            deadline=lambda info: self._deadline_callback(context, info) if self._deadline_callback else None,
            liveliness=lambda info: self._liveliness_callback(context, info) if self._liveliness_callback else None,
        )

        self._publisher = context.node.create_publisher(
            msg_type=msg_type,
            topic=topic_name,
            qos_profile=qos,
            event_callbacks=event_callbacks,
        )

    def publish(self, msg: MessageT) -> None:
        if self._publisher is None:
            raise RuntimeError("Can't publish. Publisher has not been initialised! This is an error in cake.")
        self._publisher.publish(msg)

    def publisher(self) -> RclpyPublisher:
        """Access underlying rclpy Publisher for advanced use."""
        if self._publisher is None:
            raise RuntimeError("Publisher has not been initialised! This is an error in cake.")
        return self._publisher

    def set_deadline_callback(self, callback: Callable[[Any, QoSOfferedDeadlineMissedInfo], None]):
        if self._publisher is None:
            raise RuntimeError(
                "Can't set deadline callback. Publisher has not been initialised! This is an error in cake."
            )
        self._deadline_callback = callback

    def set_liveliness_callback(self, callback: Callable[[Any, QoSLivelinessLostInfo], None]):
        if self._publisher is None:
            raise RuntimeError(
                "Can't set liveliness callback. Publisher has not been initialised! This is an error in cake."
            )
        self._liveliness_callback = callback


__all__ = ["Publisher"]
