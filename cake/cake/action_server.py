from dataclasses import dataclass
import threading

from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy

from rclpy.action import ActionServer
from rclpy.action.server import CancelResponse, GoalResponse, ServerGoalHandle

from .context import Context

from typing import Any, Callable, Generic, TypeVar

ActionT = TypeVar("ActionT")


@dataclass
class SingleGoalActionServerOptions:
    """Options for configuring a SingleGoalActionServer.

    Attributes:
        new_goals_replace_current_goal: If True, accepting a new goal will cancel
            the current active goal. If False, new goals are rejected while one is active.
        goal_validator: Function to validate goals before acceptance. Should return
            True to accept, False to reject.
    """

    new_goals_replace_current_goal: bool = False
    goal_validator: Callable[[Any], bool] = lambda goal: True


class SingleGoalActionServer(Generic[ActionT]):
    """A single-goal action server for polling-based goal execution.

    This action server only accepts one goal at a time. Users poll for the active
    goal using get_active_goal() (typically in a timer callback) and call succeed()
    or abort() when done.

    This design avoids the complexity of threaded execute callbacks and maps well
    to polling-based control loops.
    """

    _action_server: ActionServer | None = None
    _active_goal_handle: ServerGoalHandle | None = None
    _options: SingleGoalActionServerOptions | None = None
    _context: Context | None = None
    _action_name: str = ""
    _lock: threading.Lock

    def __init__(self):
        """Create an uninitialized action server.

        The action server must be initialized via _initialise() before use.
        This is typically done by generated code.
        """
        self._lock = threading.Lock()

    def _initialise(
        self,
        context: Context,
        action_type: type[ActionT],
        action_name: str,
    ) -> None:
        """Initialize the action server.

        Args:
            context: The cake context containing the ROS node.
            action_type: The action type class (e.g., example_interfaces.action.Fibonacci).
            action_name: Name of the action server.
        """
        self._context = context
        self._action_name = action_name

        # Create the underlying rclpy ActionServer with our custom callbacks
        self._action_server = ActionServer(
            context.node,
            action_type,
            action_name,
            execute_callback=self._dummy_execute,
            handle_accepted_callback=self._on_goal_accepted,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
        )

    def set_options(self, options: SingleGoalActionServerOptions) -> None:
        """Configure the action server options.

        Args:
            options: Configuration for goal validation and replacement behavior.
        """
        if self._action_server is None:
            raise RuntimeError(
                "Can't set options. Action server has not been initialised! " "This is an error in cake."
            )
        with self._lock:
            self._options = options

    def get_active_goal(self) -> Any | None:
        """Get the currently active goal, if any.

        Returns:
            The goal request if a goal is active, None if no goal is active or
            if the goal has been cancelled.
        """
        with self._lock:
            if self._active_goal_handle is None:
                return None

            # Check if goal is still active (not cancelled)
            if not self._active_goal_handle.is_active:
                # Goal was cancelled, clear it
                self._active_goal_handle = None
                return None

            return self._active_goal_handle.request

    def publish_feedback(self, feedback: Any) -> None:
        """Publish feedback for the active goal.

        Args:
            feedback: The feedback message to publish.
        """
        with self._lock:
            if self._active_goal_handle is None or not self._active_goal_handle.is_active:
                if self._context:
                    self._context.node.get_logger().warning(
                        f"Action server '{self._action_name}' tried to publish feedback but no goal is active."
                    )
                return

            self._active_goal_handle.publish_feedback(feedback)

    def succeed(self, result: Any) -> None:
        """Mark the active goal as succeeded.

        Args:
            result: The result message to send to the client.
        """
        with self._lock:
            if self._active_goal_handle is None or not self._active_goal_handle.is_active:
                if self._context:
                    self._context.node.get_logger().warning(
                        f"Action server '{self._action_name}' tried to succeed goal " f"but no goal is active."
                    )
                return

            # Get the goal UUID for looking up the result future
            goal_uuid = bytes(self._active_goal_handle.goal_id.uuid)

            # Transition goal state to succeeded
            self._active_goal_handle.succeed()

            # Build and set the result response
            result_response = self._action_server._action_type.Impl.GetResultService.Response()
            result_response.status = self._active_goal_handle.status
            result_response.result = result

            # Set the result on the Future so clients can retrieve it
            self._action_server._result_futures[goal_uuid].set_result(result_response)

            # Clear our reference
            self._active_goal_handle = None

    def abort(self, result: Any) -> None:
        """Mark the active goal as aborted.

        Args:
            result: The result message to send to the client.
        """
        with self._lock:
            if self._active_goal_handle is None or not self._active_goal_handle.is_active:
                if self._context:
                    self._context.node.get_logger().warning(
                        f"Action server '{self._action_name}' tried to abort goal but no goal is active."
                    )
                return

            # Get the goal UUID for looking up the result future
            goal_uuid = bytes(self._active_goal_handle.goal_id.uuid)

            # Transition goal state to aborted
            self._active_goal_handle.abort()

            # Build and set the result response
            result_response = self._action_server._action_type.Impl.GetResultService.Response()
            result_response.status = self._active_goal_handle.status
            result_response.result = result

            # Set the result on the Future so clients can retrieve it
            self._action_server._result_futures[goal_uuid].set_result(result_response)

            # Clear our reference
            self._active_goal_handle = None

    # Internal callback methods

    def _on_goal(self, goal_request: Any) -> GoalResponse:
        """Handle a new goal request.

        This callback decides whether to accept or reject the goal based on:
        1. Whether options are configured
        2. Goal validation
        3. Whether another goal is currently active

        Args:
            goal_request: The goal request from the client.

        Returns:
            GoalResponse.ACCEPT or GoalResponse.REJECT
        """
        with self._lock:
            # Check if options are configured
            if self._options is None:
                if self._context:
                    self._context.node.get_logger().warning(
                        f"Action server '{self._action_name}' has no options configured. "
                        f"Rejecting all goals until set_options() is called."
                    )
                return GoalResponse.REJECT

            # Validate the goal
            if not self._options.goal_validator(goal_request):
                if self._context:
                    self._context.node.get_logger().warning(
                        f"Action server '{self._action_name}' rejecting goal: " f"goal_validator returned False."
                    )
                return GoalResponse.REJECT

            # Check if there's an active goal
            if self._active_goal_handle is not None and self._active_goal_handle.is_active:
                if self._options.new_goals_replace_current_goal:
                    # Cancel current goal to make room for new one
                    if self._context:
                        self._context.node.get_logger().warning(
                            f"Action server '{self._action_name}' cancelling current goal " f"to accept new goal."
                        )
                    self._active_goal_handle.canceled()
                    self._active_goal_handle = None
                else:
                    # Reject new goal
                    if self._context:
                        self._context.node.get_logger().warning(
                            f"Action server '{self._action_name}' rejecting goal: " f"another goal is already active."
                        )
                    return GoalResponse.REJECT

            return GoalResponse.ACCEPT

    def _on_goal_accepted(self, goal_handle: ServerGoalHandle) -> None:
        """Handle a newly accepted goal.

        This callback stores the goal handle and transitions it to EXECUTING state.
        We DON'T call goal_handle.execute() because that would trigger the execute callback.

        Args:
            goal_handle: The handle for the accepted goal.
        """
        with self._lock:
            self._active_goal_handle = goal_handle

        # Transition to EXECUTING state manually (without calling execute callback)
        # This is necessary because succeed/abort can only be called from EXECUTING state
        goal_handle._update_state(_rclpy.GoalEvent.EXECUTE)

        # NOTE: We explicitly do NOT call goal_handle.execute() here.
        # That would trigger the execute callback, which we don't want.
        # The user will poll get_active_goal() and call succeed/abort manually.

    def _on_cancel(self, goal_handle: ServerGoalHandle) -> CancelResponse:
        """Handle a cancel request.

        Args:
            goal_handle: The goal handle to cancel.

        Returns:
            CancelResponse.ACCEPT
        """
        with self._lock:
            # Check if this is the active goal
            if self._active_goal_handle is not None and self._active_goal_handle.goal_id == goal_handle.goal_id:
                self._active_goal_handle = None

            return CancelResponse.ACCEPT

    def _dummy_execute(self, goal_handle: ServerGoalHandle) -> Any:
        """Dummy execute callback that should never be called.

        This is required by the ActionServer API but will never actually run
        because we don't call goal_handle.execute() in _on_goal_accepted.

        Args:
            goal_handle: The goal handle (unused).

        Returns:
            An empty result (never actually returned).
        """
        # This should never be called
        if self._context:
            self._context.node.get_logger().error(
                f"Action server '{self._action_name}' execute callback was called! "
                f"This should never happen and indicates a bug in cake."
            )
        return None


__all__ = ["SingleGoalActionServer", "SingleGoalActionServerOptions"]
