from enum import IntEnum

from rclpy.lifecycle import TransitionCallbackReturn as _RclpyTransitionCallbackReturn


class TransitionCallbackReturn(IntEnum):
    """Lifecycle transition callback return values (typed shadow of rclpy pybind enum)."""

    SUCCESS = int(_RclpyTransitionCallbackReturn.SUCCESS)
    FAILURE = int(_RclpyTransitionCallbackReturn.FAILURE)
    ERROR = int(_RclpyTransitionCallbackReturn.ERROR)

    def _to_rclpy(self) -> _RclpyTransitionCallbackReturn:
        return _RclpyTransitionCallbackReturn(self.value)


__all__ = ["TransitionCallbackReturn"]
