"""
QoS helper functions for converting string parameter values to QoS policy enums.
These functions are used when QoS fields reference parameters via ${param:name} syntax.
"""

from rclpy.qos import DurabilityPolicy, LivelinessPolicy, ReliabilityPolicy


def _to_reliability(value: str) -> ReliabilityPolicy:
    """
    Convert a string parameter value to ReliabilityPolicy enum.

    Args:
        value: The string value ("RELIABLE" or "BEST_EFFORT")

    Returns:
        The corresponding ReliabilityPolicy enum value

    Raises:
        ValueError: If the value is not a valid reliability policy
    """
    mapping = {
        "RELIABLE": ReliabilityPolicy.RELIABLE,
        "BEST_EFFORT": ReliabilityPolicy.BEST_EFFORT,
    }
    if value not in mapping:
        raise ValueError(f"Invalid reliability policy: '{value}'. Expected 'RELIABLE' or 'BEST_EFFORT'.")
    return mapping[value]


def _to_durability(value: str) -> DurabilityPolicy:
    """
    Convert a string parameter value to DurabilityPolicy enum.

    Args:
        value: The string value ("TRANSIENT_LOCAL" or "VOLATILE")

    Returns:
        The corresponding DurabilityPolicy enum value

    Raises:
        ValueError: If the value is not a valid durability policy
    """
    mapping = {
        "TRANSIENT_LOCAL": DurabilityPolicy.TRANSIENT_LOCAL,
        "VOLATILE": DurabilityPolicy.VOLATILE,
    }
    if value not in mapping:
        raise ValueError(f"Invalid durability policy: '{value}'. Expected 'TRANSIENT_LOCAL' or 'VOLATILE'.")
    return mapping[value]


def _to_liveliness(value: str) -> LivelinessPolicy:
    """
    Convert a string parameter value to LivelinessPolicy enum.

    Args:
        value: The string value ("AUTOMATIC" or "MANUAL_BY_TOPIC")

    Returns:
        The corresponding LivelinessPolicy enum value

    Raises:
        ValueError: If the value is not a valid liveliness policy
    """
    mapping = {
        "AUTOMATIC": LivelinessPolicy.AUTOMATIC,
        "MANUAL_BY_TOPIC": LivelinessPolicy.MANUAL_BY_TOPIC,
    }
    if value not in mapping:
        raise ValueError(f"Invalid liveliness policy: '{value}'. Expected 'AUTOMATIC' or 'MANUAL_BY_TOPIC'.")
    return mapping[value]
