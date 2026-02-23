from dataclasses import dataclass

from cake_example.python_node.interface import PythonNodeSession, run

from std_msgs.msg import Bool, String

from example_interfaces.action import Fibonacci

import cake
from cake import TransitionCallbackReturn

from typing import cast


@dataclass
class MySession(PythonNodeSession):
    important_number: float = 6.7
    count: int = 0


def topic_callback(sn: MySession, msg: Bool):
    response = f"Got message: {msg.data}"
    sn.logger.info(response)
    sn.publishers.a_topic.publish(String(data=response))


def action_func(sn: MySession):
    sn.logger.info("Checking for action!")
    active_goal = sn.actions.my_action_two.get_active_goal()
    if active_goal is None:
        sn.count = 0

    if active_goal is not None:
        active_goal = cast(Fibonacci.Goal, active_goal)
        sn.logger.info(f"Got goal: {active_goal.order}")
        sn.count += 1

    if sn.count > 10:
        result = Fibonacci.Result(sequence=[1, 2, 3, 4])
        sn.actions.my_action_two.succeed(result)


def on_configure(sn: MySession) -> TransitionCallbackReturn:
    sn.logger.info("Hello from python cake!")
    sn.logger.info(f"The parameter is: {sn.params.special_number}. The session value is: {sn.important_number}")
    sn.subscribers.another_topic.set_callback(topic_callback)
    sn.actions.my_action_two.set_options(cake.SingleGoalActionServerOptions(new_goals_replace_current_goal=True))
    cake.create_timer(sn, 1, action_func)
    return TransitionCallbackReturn.SUCCESS


if __name__ == "__main__":
    run(MySession, on_configure)
