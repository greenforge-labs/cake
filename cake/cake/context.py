from dataclasses import dataclass, field

from threading import Thread

from rclpy.node import Node
from rclpy.timer import Timer


@dataclass(kw_only=True)
class Context:
    node: Node
    timers: list[Timer] = field(default_factory=list)
    threads: list[Thread] = field(default_factory=list)

    @property
    def logger(self):
        return self.node.get_logger()


__all__ = ["Context"]
