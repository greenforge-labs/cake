from __future__ import annotations

from rclpy.lifecycle import LifecycleNode, LifecycleState
from rclpy.lifecycle import TransitionCallbackReturn as _RclpyTCR

from .session import Session
from .transition import TransitionCallbackReturn

from typing import Callable, Generic, TypeVar

_SUCCESS = int(_RclpyTCR.SUCCESS)

_SessionT = TypeVar("_SessionT", bound=Session)


def _to_rclpy(value: TransitionCallbackReturn | _RclpyTCR) -> _RclpyTCR:
    """Convert any transition return (our IntEnum or pybind) to the pybind type rclpy expects."""
    if isinstance(value, _RclpyTCR):
        return value
    return _RclpyTCR(int(value))


class _CakeLifecycleNode(LifecycleNode):
    """Internal lifecycle node that delegates lifecycle callbacks to BaseNode."""

    def __init__(
        self,
        node_name: str,
        *,
        on_configure: Callable[[], TransitionCallbackReturn],
        on_activate: Callable[[], TransitionCallbackReturn],
        on_deactivate: Callable[[], TransitionCallbackReturn],
        on_cleanup: Callable[[], TransitionCallbackReturn],
        on_shutdown: Callable[[], TransitionCallbackReturn],
        on_error: Callable[[], TransitionCallbackReturn],
        **kwargs,
    ) -> None:
        self._on_configure_cb = on_configure
        self._on_activate_cb = on_activate
        self._on_deactivate_cb = on_deactivate
        self._on_cleanup_cb = on_cleanup
        self._on_shutdown_cb = on_shutdown
        self._on_error_cb = on_error
        super().__init__(node_name, **kwargs)

    @property
    def current_state(self) -> int:
        """Returns state ID int, e.g. State.PRIMARY_STATE_ACTIVE."""
        return self._state_machine.current_state[0]

    def on_configure(self, state: LifecycleState) -> _RclpyTCR:
        return _to_rclpy(self._on_configure_cb())

    def on_activate(self, state: LifecycleState) -> _RclpyTCR:
        return _to_rclpy(self._on_activate_cb())

    def on_deactivate(self, state: LifecycleState) -> _RclpyTCR:
        return _to_rclpy(self._on_deactivate_cb())

    def on_cleanup(self, state: LifecycleState) -> _RclpyTCR:
        return _to_rclpy(self._on_cleanup_cb())

    def on_shutdown(self, state: LifecycleState) -> _RclpyTCR:
        return _to_rclpy(self._on_shutdown_cb())

    def on_error(self, state: LifecycleState) -> _RclpyTCR:
        return _to_rclpy(self._on_error_cb())


class BaseNode(Generic[_SessionT]):
    """Lifecycle node wrapper using composition (mirrors C++ cake::BaseNode).

    Wraps a _CakeLifecycleNode and orchestrates session creation/destruction
    around lifecycle transitions.
    """

    def __init__(
        self,
        node_name: str,
        session_type: type[_SessionT],
        on_configure: Callable[[_SessionT], TransitionCallbackReturn],
        *,
        on_activate: Callable[[_SessionT], TransitionCallbackReturn] | None = None,
        on_deactivate: Callable[[_SessionT], TransitionCallbackReturn] | None = None,
        on_cleanup: Callable[[_SessionT], TransitionCallbackReturn] | None = None,
        on_shutdown: Callable[[_SessionT], None] | None = None,
        **kwargs,
    ) -> None:
        self._node = _CakeLifecycleNode(
            node_name,
            on_configure=self._handle_configure,
            on_activate=self._handle_activate,
            on_deactivate=self._handle_deactivate,
            on_cleanup=self._handle_cleanup,
            on_shutdown=self._handle_shutdown,
            on_error=self._handle_error,
            **kwargs,
        )
        self._session_type = session_type
        self._session: _SessionT | None = None
        self._on_configure_cb = on_configure
        self._on_activate_cb = on_activate
        self._on_deactivate_cb = on_deactivate
        self._on_cleanup_cb = on_cleanup
        self._on_shutdown_cb = on_shutdown

    @property
    def node(self) -> _CakeLifecycleNode:
        return self._node

    def _handle_configure(self) -> TransitionCallbackReturn:
        self._session = self._create_session(self._node)
        result = self._on_configure_cb(self._session)
        if int(result) != _SUCCESS:
            self._destroy_entities(self._session)
            self._session = None
        return result

    def _handle_activate(self) -> TransitionCallbackReturn:
        assert self._session is not None
        if self._on_activate_cb is not None:
            result = self._on_activate_cb(self._session)
            if int(result) != _SUCCESS:
                return result
        self._activate_entities(self._session)
        return TransitionCallbackReturn.SUCCESS

    def _handle_deactivate(self) -> TransitionCallbackReturn:
        assert self._session is not None
        if self._on_deactivate_cb is not None:
            result = self._on_deactivate_cb(self._session)
            if int(result) != _SUCCESS:
                return result
        self._deactivate_entities(self._session)
        return TransitionCallbackReturn.SUCCESS

    def _handle_cleanup(self) -> TransitionCallbackReturn:
        assert self._session is not None
        if self._on_cleanup_cb is not None:
            result = self._on_cleanup_cb(self._session)
            if int(result) != _SUCCESS:
                return result
        self._destroy_entities(self._session)
        self._session = None
        return TransitionCallbackReturn.SUCCESS

    def _handle_shutdown(self) -> TransitionCallbackReturn:
        if self._session is not None:
            if self._on_shutdown_cb is not None:
                self._on_shutdown_cb(self._session)
            self._destroy_entities(self._session)
            self._session = None
        return TransitionCallbackReturn.SUCCESS

    def _handle_error(self) -> TransitionCallbackReturn:
        if self._session is not None:
            self._destroy_entities(self._session)
            self._session = None
        return TransitionCallbackReturn.FAILURE

    def _create_session(self, node: _CakeLifecycleNode) -> _SessionT:
        raise NotImplementedError

    def _activate_entities(self, sn: _SessionT) -> None:
        pass

    def _deactivate_entities(self, sn: _SessionT) -> None:
        pass

    def _destroy_entities(self, sn: _SessionT) -> None:
        pass


__all__ = ["BaseNode"]
