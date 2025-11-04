from dataclasses import dataclass, field

from threading import Thread

import rclpy
from rclpy.node import Node
from rclpy.timer import Timer


@dataclass(kw_only=True)
class Context:
    node: Node
    timers: list[Timer] = field(default_factory=list)
    threads: list[Thread] = field(default_factory=list)
