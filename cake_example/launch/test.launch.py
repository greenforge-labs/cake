import clingwrap as cw


def generate_launch_description():
    l = cw.LaunchBuilder()

    with l.namespace("cleaning"):
        l.node("cake_example", "my_node")
        l.node("cake_example", "python_node")

    return l
