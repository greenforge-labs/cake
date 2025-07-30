import rclpy
from rclpy.node import Node

from typing import Callable, List, Optional


def run_functional_node(init_func: Callable[[Node], None], node_name: str, *, args: Optional[List[str]] = None):
    rclpy.init(args=args)
