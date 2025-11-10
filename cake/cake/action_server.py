from dataclasses import dataclass
from enum import Enum
import threading

from rclpy.node import Node

from rclpy.action.server import ActionServer, CancelResponse, GoalResponse, ServerGoalHandle

from typing import Any, Callable, Generic, Type, TypeVar

ActionT = TypeVar("ActionT")
GoalT = TypeVar("GoalT")
ResultT = TypeVar("ResultT")
FeedbackT = TypeVar("FeedbackT")


class _Yield:
    """Helper class to yield control back to the executor without creating an async generator."""

    def __await__(self):
        yield
        return None


@dataclass
class SingleGoalActionServerOptions(Generic[GoalT]):
    new_goals_replace_current_goal: bool = False
    goal_validator: Callable[[GoalT], bool] = lambda goal: True


class SingleGoalActionServer(Generic[ActionT, GoalT, ResultT, FeedbackT]):
    """
    Wrapper around ActionServer that allows external code to control execution.

    This class tracks one active goal at a time. When a new goal is accepted,
    any previous active goal is automatically aborted. The execute callback
    yields control back to the executor while waiting for external code to call
    succeed() or abort() on the wrapper. Cancellation requests from ROS clients
    are handled automatically.
    """

    _active_goal_handle: ServerGoalHandle | None = None
    _result_event: threading.Event | None = None
    _result_storage: dict[threading.Event, Any]  # Maps event to result for that goal
    _lock: threading.Lock
    _node: Node
    _action_type: Type[ActionT]
    _action_name: str
    _options: SingleGoalActionServerOptions[GoalT] | None

    def __init__(
        self,
        node: Node,
        action_type: Type[ActionT],
        action_name: str,
        options: SingleGoalActionServerOptions[GoalT] | None = None,
        **kwargs,
    ):
        """
        Create a single-goal action server with external execution control.

        :param node: The ROS node
        :param action_type: Type of the action
        :param action_name: Name of the action
        :param options: Optional configuration for the action server
        :param kwargs: Additional arguments passed to ActionServer
        """
        self._lock = threading.Lock()
        self._result_storage = {}
        self._node = node
        self._action_type = action_type
        self._action_name = action_name
        self._options = options

        # Override callbacks
        if "goal_callback" not in kwargs:
            kwargs["goal_callback"] = self._default_goal_callback
        if "handle_accepted_callback" not in kwargs:
            kwargs["handle_accepted_callback"] = self._handle_accepted_callback
        if "cancel_callback" not in kwargs:
            kwargs["cancel_callback"] = self._cancel_callback

        # Create the underlying action server with our execute callback
        self._action_server = ActionServer(node, action_type, action_name, self._execute_callback, **kwargs)

    def set_options(self, options: SingleGoalActionServerOptions[GoalT]) -> None:
        """
        Set or update the options for this action server.

        :param options: Configuration options for the action server
        """
        self._options = options

    def _signal_result(self, event: threading.Event, result: ResultT) -> None:
        """
        Store result and signal the event to wake the waiting execute callback.

        :param event: The event to signal
        :param result: The result to store for this goal
        """
        self._result_storage[event] = result
        event.set()

    def _default_goal_callback(self, goal_request: GoalT) -> GoalResponse:
        """
        Default goal callback that uses options to decide acceptance.

        This callback checks:
        1. If options are configured
        2. If the goal passes validation
        3. If there's an active goal and whether to replace it
        """
        # Check if options are configured
        if self._options is None:
            self._node.get_logger().warn(
                f"Action server '{self._action_name}': Rejecting goal, "
                "options not configured. Call set_options() to configure."
            )
            return GoalResponse.REJECT

        # Validate the goal
        if not self._options.goal_validator(goal_request):
            self._node.get_logger().warn(f"Action server '{self._action_name}': Rejecting goal, goal is invalid")
            return GoalResponse.REJECT

        # Check if there's an active goal
        with self._lock:
            if self._active_goal_handle is not None and self._active_goal_handle.is_active:
                if not self._options.new_goals_replace_current_goal:
                    self._node.get_logger().warn(
                        f"Action server '{self._action_name}': " "Rejecting goal, another goal is active"
                    )
                    return GoalResponse.REJECT

        self._node.get_logger().info(f"Action server '{self._action_name}': Accepting goal")
        return GoalResponse.ACCEPT

    def _handle_accepted_callback(self, goal_handle: ServerGoalHandle) -> None:
        """Handle newly accepted goals - abort any previous active goal if configured."""
        with self._lock:
            # If there's an active goal and we're replacing it, abort it
            if self._active_goal_handle is not None and self._active_goal_handle.is_active:
                if self._options and self._options.new_goals_replace_current_goal:
                    self._node.get_logger().warn(
                        f"Action server '{self._action_name}': Aborting current goal for new goal"
                    )
                    # Wake up the waiting execute callback with abort state
                    self._active_goal_handle.abort()
                    if self._result_event is not None:
                        self._signal_result(self._result_event, self._action_type.Result())

            # Set the new goal as active
            self._active_goal_handle = goal_handle
            self._result_event = threading.Event()

            self._node.get_logger().info(f"Action server '{self._action_name}': Goal accepted")

        # Execute the goal
        goal_handle.execute()

    def _cancel_callback(self, goal_handle: ServerGoalHandle) -> CancelResponse:
        """
        Handle cancel requests by accepting them and waking the execute callback.

        The action server will transition the goal to CANCELING state after this returns.
        The execute callback will then detect the cancellation and finalize it.

        :param goal_handle: The goal handle to cancel
        :return: CancelResponse indicating whether to accept the cancellation
        """
        self._node.get_logger().info(f"Action server '{self._action_name}': Received request to cancel goal")

        with self._lock:
            if self._active_goal_handle and goal_handle.goal_id == self._active_goal_handle.goal_id:
                # Wake up the waiting execute callback with empty result
                # Don't call canceled() here - the goal will be in CANCELING state
                # after this callback returns, and execute callback will finalize it
                if self._result_event is not None:
                    self._signal_result(self._result_event, self._action_type.Result())
                self._node.get_logger().info(f"Action server '{self._action_name}': Accepting cancel request")

        return CancelResponse.ACCEPT

    async def _execute_callback(self, goal_handle: ServerGoalHandle) -> ResultT:
        """
        Internal execute callback that waits for external code to complete the goal.

        This yields control back to the executor on each spin cycle, allowing other
        callbacks to run while waiting for succeed() or abort() to be called.
        If a cancellation is requested by a ROS client, this will also be handled.
        """
        self._node.get_logger().info("Execute callback waiting for external completion...")

        # Capture local references to this goal's event and result storage
        # This prevents issues when a new goal overwrites the shared state
        if self._result_event is None:
            raise RuntimeError("Result event not initialized")

        result_event = self._result_event

        # Wait for the result to be set by succeed/abort or a cancel request
        # Yield control back to executor while waiting, allowing other work to proceed
        while not result_event.is_set():
            await _Yield()

        # Perform the appropriate state transition based on the goal state
        with self._lock:
            # If cancelled via ROS cancel request, the goal will be in CANCELING state
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()

            # Only clear active goal if it's still this goal
            if self._active_goal_handle == goal_handle:
                self._active_goal_handle = None

            # Get result from storage for this specific goal
            result = self._result_storage.get(result_event, self._action_type.Result())
            # Clean up storage
            self._result_storage.pop(result_event, None)

            self._node.get_logger().info("Execute callback returning result")
            return result

    def get_active_goal_handle(self) -> ServerGoalHandle | None:
        """Get the currently active goal handle, or None if no goal is active."""
        with self._lock:
            return self._active_goal_handle

    def get_active_goal(self) -> GoalT | None:
        """
        Get the currently active goal request, or None if no goal is active.

        :return: The goal request object, or None
        """
        with self._lock:
            if self._active_goal_handle is None:
                return None
            return self._active_goal_handle.request

    def succeed(self, result: ResultT | None = None) -> None:
        """
        Mark the active goal as succeeded and provide the result.

        :param result: The result to return (or None for empty result)
        """
        with self._lock:
            if self._active_goal_handle is None or not self._active_goal_handle.is_active:
                self._node.get_logger().warn(f"Action server '{self._action_name}': Cannot succeed, no active goal")
                return

            self._active_goal_handle.succeed()
            if self._result_event is not None:
                self._signal_result(self._result_event, result if result is not None else self._action_type.Result())

    def abort(self, result: ResultT | None = None) -> None:
        """
        Mark the active goal as aborted and provide the result.

        :param result: The result to return (or None for empty result)
        """
        with self._lock:
            if self._active_goal_handle is None or not self._active_goal_handle.is_active:
                self._node.get_logger().warn(f"Action server '{self._action_name}': Cannot abort, no active goal")
                return

            self._active_goal_handle.abort()
            if self._result_event is not None:
                self._signal_result(self._result_event, result if result is not None else self._action_type.Result())

    def publish_feedback(self, feedback: FeedbackT) -> None:
        """Publish feedback for the active goal."""
        with self._lock:
            if self._active_goal_handle is not None and self._active_goal_handle.is_active:
                self._active_goal_handle.publish_feedback(feedback)

    def __del__(self) -> None:
        """Destructor that aborts any active goal on cleanup."""
        with self._lock:
            if self._active_goal_handle is not None and self._active_goal_handle.is_active:
                # Wake up the waiting execute callback with abort state
                self._active_goal_handle.abort()
                if self._result_event is not None:
                    self._signal_result(self._result_event, self._action_type.Result())

    def destroy(self) -> None:
        """Destroy the underlying action server."""
        self._action_server.destroy()
