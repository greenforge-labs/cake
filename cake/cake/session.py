from dataclasses import dataclass, field

from rclpy.lifecycle import LifecycleNode
from rclpy.timer import Timer


@dataclass(kw_only=True)
class Session:
    node: LifecycleNode
    timers: list[Timer] = field(default_factory=list)

    @property
    def logger(self):
        return self.node.get_logger()


__all__ = ["Session"]
