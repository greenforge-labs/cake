from rclpy.event_handler import QoSLivelinessChangedInfo, QoSRequestedDeadlineMissedInfo

from lifecycle_msgs.msg import State

from .session import Session
from .subscriber import Subscriber


def attach_default_qos_handlers(subscriber: Subscriber) -> None:
    topic: str = subscriber.subscription().topic_name

    def _deadline_callback(session: Session, info: QoSRequestedDeadlineMissedInfo) -> None:
        if session.node.current_state != State.PRIMARY_STATE_ACTIVE:
            return
        session.node.get_logger().error(f"Subscriber '{topic}': deadline missed \u2014 deactivating node")
        session.node.trigger_deactivate()

    def _liveliness_callback(session: Session, info: QoSLivelinessChangedInfo) -> None:
        # Deactivate if there are no alive publishers remaining — covers both lease expiry and publisher removal.
        if info.alive_count > 0:
            return
        if session.node.current_state != State.PRIMARY_STATE_ACTIVE:
            return
        session.node.get_logger().error(f"Subscriber '{topic}': topic has no alive publishers \u2014 deactivating node")
        session.node.trigger_deactivate()

    subscriber.set_deadline_callback(_deadline_callback)
    subscriber.set_liveliness_callback(_liveliness_callback)
