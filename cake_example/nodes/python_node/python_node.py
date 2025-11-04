# from cake_example.python_node import PythonNodeContext, run
from cake.dummy_python_node import PythonNodeContext, run


class Context(PythonNodeContext):
    important_number: float = 6.7


def init(ctx: Context):
    ctx.node.get_logger().info("Hello from python cake!")
    ctx.node.get_logger().info(
        f"The parameter is: {ctx.params.special_number}. The context value is: {ctx.important_number}"
    )


if __name__ == "__main__":
    run(Context, init)
