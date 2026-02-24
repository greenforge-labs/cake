from rclpy.event_handler import QoSLivelinessChangedInfo, QoSRequestedDeadlineMissedInfo, SubscriptionEventCallbacks
from rclpy.qos import QoSProfile
from rclpy.subscription import Subscription

from lifecycle_msgs.msg import State

from .session import Session

from typing import Any, Callable, Generic, Optional, TypeVar, cast

MessageT = TypeVar("MessageT")


def get_no_callback_warning_logger(topic_name: str) -> Callable[[Session, Any], None]:
    def inner(sn: Session, msg: Any):
        sn.node.get_logger().warning(
            f"Subscriber {topic_name} received message but no callback configured. Call set_callback()."
        )

    return inner


class Subscriber(Generic[MessageT]):
    _subscription: Subscription | None = None
    _callback: Callable[[Any, MessageT], None]
    _deadline_callback: Optional[Callable[[Any, QoSRequestedDeadlineMissedInfo], None]] = None
    _liveliness_callback: Optional[Callable[[Any, QoSLivelinessChangedInfo], None]] = None

    def _initialise(
        self,
        session: Session,
        msg_type: type[MessageT],
        topic_name: str,
        qos: QoSProfile | int,
    ) -> None:
        self._callback = get_no_callback_warning_logger(topic_name)

        # Create event callbacks that delegate to optional user callbacks
        event_callbacks = SubscriptionEventCallbacks(
            deadline=lambda info: self._deadline_callback(session, info) if self._deadline_callback else None,
            liveliness=lambda info: self._liveliness_callback(session, info) if self._liveliness_callback else None,
        )

        def guarded_callback(msg):
            if session.node.current_state != State.PRIMARY_STATE_ACTIVE:
                return
            self._callback(session, cast(MessageT, msg))

        self._subscription = session.node.create_subscription(
            msg_type=msg_type,
            topic=topic_name,
            callback=guarded_callback,
            qos_profile=qos,
            event_callbacks=event_callbacks,
        )

    def _destroy(self, node) -> None:
        if self._subscription is not None:
            node.destroy_subscription(self._subscription)
            self._subscription = None

    def set_callback(self, callback: Callable[[Any, MessageT], None]):
        if self._subscription is None:
            raise RuntimeError("Can't set callback. Subscriber has not been initialised! This is an error in cake.")
        self._callback = callback

    def set_deadline_callback(self, callback: Callable[[Any, QoSRequestedDeadlineMissedInfo], None]):
        if self._subscription is None:
            raise RuntimeError(
                "Can't set deadline callback. Subscriber has not been initialised! This is an error in cake."
            )
        self._deadline_callback = callback

    def set_liveliness_callback(self, callback: Callable[[Any, QoSLivelinessChangedInfo], None]):
        if self._subscription is None:
            raise RuntimeError(
                "Can't set liveliness callback. Subscriber has not been initialised! This is an error in cake."
            )
        self._liveliness_callback = callback

    def subscription(self) -> Subscription:
        """Access underlying rclpy Subscription for advanced use."""
        if self._subscription is None:
            raise RuntimeError("Subscriber has not been initialised! This is an error in cake.")
        return self._subscription


__all__ = ["Subscriber"]
