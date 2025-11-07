import asyncio
from dataclasses import dataclass
import threading

from rclpy.action.server import ActionServer, CancelResponse, GoalResponse

from typing import Any, Callable


@dataclass
class SingleGoalActionServerOptions:
    new_goals_replace_current_goal: bool = False
    goal_validator: Callable[[Any], bool] = lambda goal: True


class SingleGoalActionServer:
    """
    Wrapper around ActionServer that allows external code to control execution.

    This class tracks one active goal at a time. When a new goal is accepted,
    any previous active goal is automatically aborted. The execute callback
    blocks until external code calls succeed(), abort(), or canceled() on the
    wrapper, allowing processing to happen elsewhere while still maintaining
    access to the goal handle for cancellation checks.
    """

    def __init__(self, node, action_type, action_name, options: SingleGoalActionServerOptions | None = None, **kwargs):
        """
        Create a single-goal action server with external execution control.

        :param node: The ROS node
        :param action_type: Type of the action
        :param action_name: Name of the action
        :param options: Optional configuration for the action server
        :param kwargs: Additional arguments passed to ActionServer
        """
        self._active_goal_handle: Any | None = None
        self._result_event: threading.Event | None = None
        self._result: Any | None = None
        self._lock = threading.Lock()
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

    def set_options(self, options: SingleGoalActionServerOptions) -> None:
        """
        Set or update the options for this action server.

        :param options: Configuration options for the action server
        """
        self._options = options

    def _default_goal_callback(self, goal_request):
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

    def _handle_accepted_callback(self, goal_handle):
        """Handle newly accepted goals - cancel any previous active goal if configured."""
        with self._lock:
            # If there's an active goal and we're replacing it, cancel it
            if self._active_goal_handle is not None and self._active_goal_handle.is_active:
                if self._options and self._options.new_goals_replace_current_goal:
                    self._node.get_logger().warn(f"Action server '{self._action_name}': Canceling current goal")
                    self._active_goal_handle.canceled()
                    # Wake up the waiting execute callback with empty result
                    if self._result_event is not None:
                        self._result = self._action_type.Result()
                        self._result_event.set()

            # Set the new goal as active
            self._active_goal_handle = goal_handle
            self._result_event = threading.Event()
            self._result = None

            self._node.get_logger().info(f"Action server '{self._action_name}': Goal accepted")

        # Execute the goal
        goal_handle.execute()

    def _cancel_callback(self, goal_handle):
        """
        Handle cancel requests by automatically canceling the active goal.

        :param goal_handle: The goal handle to cancel
        :return: CancelResponse indicating whether to accept the cancellation
        """
        self._node.get_logger().info(f"Action server '{self._action_name}': Received request to cancel goal")

        with self._lock:
            if self._active_goal_handle and goal_handle.goal_id == self._active_goal_handle.goal_id:
                self._active_goal_handle.canceled()
                # Wake up the waiting execute callback with empty result
                if self._result_event is not None:
                    self._result = self._action_type.Result()
                    self._result_event.set()
                self._active_goal_handle = None
                self._node.get_logger().info(f"Action server '{self._action_name}': Goal canceled")

        return CancelResponse.ACCEPT

    async def _execute_callback(self, goal_handle):
        """
        Internal execute callback that waits for external code to complete the goal.

        This is a no-op that just blocks until succeed/abort/canceled is called.
        """
        self._node.get_logger().info("Execute callback waiting for external completion...")

        # Wait for the result to be set by succeed/abort/canceled
        # We need to wait in a way that's compatible with async
        loop = asyncio.get_event_loop()
        await loop.run_in_executor(None, self._result_event.wait)

        with self._lock:
            result = self._result if self._result is not None else self._action_type.Result()
            self._node.get_logger().info("Execute callback returning result")
            return result

    def get_active_goal_handle(self):
        """Get the currently active goal handle, or None if no goal is active."""
        with self._lock:
            return self._active_goal_handle

    def get_active_goal(self):
        """
        Get the currently active goal request, or None if no goal is active.

        :return: The goal request object, or None
        """
        with self._lock:
            if self._active_goal_handle is None:
                return None
            return self._active_goal_handle.request

    def succeed(self, result=None) -> None:
        """
        Mark the active goal as succeeded and provide the result.

        :param result: The result to return (or None for empty result)
        """
        with self._lock:
            if self._active_goal_handle is None or not self._active_goal_handle.is_active:
                self._node.get_logger().warn(f"Action server '{self._action_name}': Cannot succeed, no active goal")
                return

            self._active_goal_handle.succeed()
            self._result = result if result is not None else self._action_type.Result()
            self._result_event.set()
            self._active_goal_handle = None

    def abort(self, result=None) -> None:
        """
        Mark the active goal as aborted and provide the result.

        :param result: The result to return (or None for empty result)
        """
        with self._lock:
            if self._active_goal_handle is None or not self._active_goal_handle.is_active:
                self._node.get_logger().warn(f"Action server '{self._action_name}': Cannot abort, no active goal")
                return

            self._active_goal_handle.abort()
            self._result = result if result is not None else self._action_type.Result()
            self._result_event.set()
            self._active_goal_handle = None

    def cancel(self, result=None) -> None:
        """
        Mark the active goal as canceled and provide the result.

        :param result: The result to return (or None for empty result)
        """
        with self._lock:
            if self._active_goal_handle is None or not self._active_goal_handle.is_active:
                self._node.get_logger().warn(f"Action server '{self._action_name}': Cannot cancel, no active goal")
                return

            self._active_goal_handle.canceled()
            self._result = result if result is not None else self._action_type.Result()
            self._result_event.set()
            self._active_goal_handle = None

    def publish_feedback(self, feedback):
        """Publish feedback for the active goal."""
        with self._lock:
            if self._active_goal_handle is not None and self._active_goal_handle.is_active:
                self._active_goal_handle.publish_feedback(feedback)

    def __del__(self):
        """Destructor that aborts any active goal on cleanup."""
        with self._lock:
            if self._active_goal_handle is not None and self._active_goal_handle.is_active:
                self._active_goal_handle.abort()
                # Wake up the waiting execute callback with empty result
                if self._result_event is not None:
                    self._result = self._action_type.Result()
                    self._result_event.set()
                self._active_goal_handle = None

    def destroy(self):
        """Destroy the underlying action server."""
        self._action_server.destroy()


def create_single_goal_action_server(
    node, action_type, action_name, options: SingleGoalActionServerOptions | None = None, **kwargs
) -> SingleGoalActionServer:
    """
    Factory function to create a SingleGoalActionServer.

    :param node: The ROS node
    :param action_type: Type of the action
    :param action_name: Name of the action
    :param options: Optional configuration for the action server
    :param kwargs: Additional arguments passed to ActionServer
    :return: A new SingleGoalActionServer instance
    """
    return SingleGoalActionServer(node, action_type, action_name, options, **kwargs)
