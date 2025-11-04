#!/usr/bin/env python3

"""
Code generator for cake ROS2 node interfaces.
Parses interface.yaml and generates C++ header with publishers, subscribers, context, and base node class.
"""

import argparse
from pathlib import Path
import re
import sys
import tempfile

from jinja2 import Environment, FileSystemLoader
import yaml

from typing import Any, Dict, List, Set


def camel_to_snake(name: str) -> str:
    """
    Convert CamelCase or PascalCase to snake_case.
    Example: AddTwoInts -> add_two_ints, String -> string
    """
    # Insert underscore before uppercase letters that follow lowercase letters or digits
    s1 = re.sub("(.)([A-Z][a-z]+)", r"\1_\2", name)
    # Insert underscore before uppercase letters that follow lowercase or digits
    return re.sub("([a-z0-9])([A-Z])", r"\1_\2", s1).lower()


def ros_type_to_cpp(ros_type: str) -> str:
    """
    Convert ROS message type from slash notation to C++ namespace notation.
    Example: std_msgs/msg/String -> std_msgs::msg::String
    """
    return ros_type.replace("/", "::")


def ros_type_to_include(ros_type: str) -> str:
    """
    Convert ROS message type to include path.
    Example: std_msgs/msg/String -> std_msgs/msg/string.hpp
    Example: geometry_msgs/msg/TwistStamped -> geometry_msgs/msg/twist_stamped.hpp
    """
    parts = ros_type.split("/")
    if len(parts) >= 3:
        # Convert last part (message name) from PascalCase to snake_case
        parts[-1] = camel_to_snake(parts[-1])
    return "/".join(parts) + ".hpp"


def topic_to_field_name(topic: str) -> str:
    """
    Convert topic name to valid C++ identifier.
    Example: /cmd_vel -> cmd_vel, /robot/status -> robot_status
    """
    return topic.replace("/", "_").lstrip("_")


def generate_qos_code(qos_spec: Any) -> str:
    """
    Generate C++ QoS code from YAML QoS specification.

    Supports:
    - Integer: 10 -> "10"
    - String (predefined profile): "SensorDataQoS" -> "rclcpp::SensorDataQoS()"
    - Dict (custom parameters): {reliability: reliable, depth: 10} -> "rclcpp::QoS(10).reliable()..."
    """
    # Backward compatible: integer
    if isinstance(qos_spec, int):
        return str(qos_spec)

    # Predefined profile: string
    if isinstance(qos_spec, str):
        return f"rclcpp::{qos_spec}()"

    # Custom parameters: dict
    if isinstance(qos_spec, dict):
        # Check if using profile with overrides
        if "profile" in qos_spec:
            base_qos = f"rclcpp::{qos_spec['profile']}()"
            params = {k: v for k, v in qos_spec.items() if k != "profile"}
        else:
            # Start with depth if provided, otherwise default to 10
            depth = qos_spec.get("depth", 10)
            base_qos = f"rclcpp::QoS({depth})"
            params = {k: v for k, v in qos_spec.items() if k != "depth"}

        # Build chain of method calls
        methods = []

        # Reliability
        if "reliability" in params:
            if params["reliability"] == "reliable":
                methods.append(".reliable()")
            elif params["reliability"] == "best_effort":
                methods.append(".best_effort()")

        # Durability
        if "durability" in params:
            if params["durability"] == "volatile":
                methods.append(".durability_volatile()")
            elif params["durability"] == "transient_local":
                methods.append(".transient_local()")

        # History
        if "history" in params:
            if params["history"] == "keep_last":
                depth_val = params.get("depth", 10)
                methods.append(f".keep_last({depth_val})")
            elif params["history"] == "keep_all":
                methods.append(".keep_all()")
        elif "depth" in params:
            # Depth specified without explicit history (applies to profiles with overrides)
            methods.append(f".keep_last({params['depth']})")

        # Deadline
        if "deadline" in params:
            deadline = params["deadline"]
            if isinstance(deadline, dict):
                sec = deadline.get("sec", 0)
                nsec = deadline.get("nsec", 0)
                methods.append(f".deadline(rclcpp::Duration({sec}, {nsec}))")

        # Lifespan
        if "lifespan" in params:
            lifespan = params["lifespan"]
            if isinstance(lifespan, dict):
                sec = lifespan.get("sec", 0)
                nsec = lifespan.get("nsec", 0)
                methods.append(f".lifespan(rclcpp::Duration({sec}, {nsec}))")

        # Liveliness
        if "liveliness" in params:
            if params["liveliness"] == "automatic":
                methods.append(".liveliness(rclcpp::LivelinessPolicy::Automatic)")
            elif params["liveliness"] == "manual_by_topic":
                methods.append(".liveliness(rclcpp::LivelinessPolicy::ManualByTopic)")

        return base_qos + "".join(methods)

    # Default fallback
    return "10"


def prepare_publishers(publishers_raw: List[Dict[str, Any]]) -> List[Dict[str, str]]:
    """
    Prepare publisher data for template rendering.
    Converts raw YAML data into template-ready format.
    """
    publishers = []
    for pub in publishers_raw:
        if not pub.get("manually_created", False):
            publishers.append(
                {
                    "topic": pub["topic"],
                    "msg_type": ros_type_to_cpp(pub["type"]),
                    "field_name": topic_to_field_name(pub["topic"]),
                    "qos_code": generate_qos_code(pub.get("qos", 10)),
                }
            )
    return publishers


def prepare_subscribers(subscribers_raw: List[Dict[str, Any]]) -> List[Dict[str, str]]:
    """
    Prepare subscriber data for template rendering.
    Converts raw YAML data into template-ready format.
    """
    subscribers = []
    for sub in subscribers_raw:
        if not sub.get("manually_created", False):
            subscribers.append(
                {
                    "topic": sub["topic"],
                    "msg_type": ros_type_to_cpp(sub["type"]),
                    "field_name": topic_to_field_name(sub["topic"]),
                    "qos_code": generate_qos_code(sub.get("qos", 10)),
                }
            )
    return subscribers


def ros_type_to_service_include(ros_type: str) -> str:
    """
    Convert ROS service type to include path.
    Example: example_interfaces/srv/AddTwoInts -> example_interfaces/srv/add_two_ints.hpp
    """
    parts = ros_type.split("/")
    if len(parts) >= 3:
        # Convert last part (service name) from PascalCase to snake_case
        parts[-1] = camel_to_snake(parts[-1])
    return "/".join(parts) + ".hpp"


def service_to_field_name(service_name: str) -> str:
    """
    Convert service name to valid C++ identifier.
    Example: /add_two_ints -> add_two_ints, /robot/compute -> robot_compute
    """
    return service_name.replace("/", "_").lstrip("_")


def ros_type_to_action_include(ros_type: str) -> str:
    """
    Convert ROS action type to include path.
    Example: example_interfaces/action/Fibonacci -> example_interfaces/action/fibonacci.hpp
    """
    parts = ros_type.split("/")
    if len(parts) >= 3:
        # Convert last part (action name) from PascalCase to snake_case
        parts[-1] = camel_to_snake(parts[-1])
    return "/".join(parts) + ".hpp"


def action_to_field_name(action_name: str) -> str:
    """
    Convert action name to valid C++ identifier.
    Example: /fibonacci -> fibonacci, /robot/compute -> robot_compute
    """
    return action_name.replace("/", "_").lstrip("_")


def prepare_services(services_raw: List[Dict[str, Any]]) -> List[Dict[str, str]]:
    """
    Prepare service data for template rendering.
    Converts raw YAML data into template-ready format.
    """
    services = []
    for srv in services_raw:
        if not srv.get("manually_created", False):
            # Only generate QoS code if explicitly specified
            qos_code = None
            if "qos" in srv:
                qos_code = generate_qos_code(srv["qos"])

            services.append(
                {
                    "name": srv["name"],
                    "service_type": ros_type_to_cpp(srv["type"]),
                    "field_name": service_to_field_name(srv["name"]),
                    "qos_code": qos_code,
                }
            )
    return services


def prepare_service_clients(service_clients_raw: List[Dict[str, Any]]) -> List[Dict[str, str]]:
    """
    Prepare service client data for template rendering.
    Converts raw YAML data into template-ready format.
    """
    service_clients = []
    for client in service_clients_raw:
        if not client.get("manually_created", False):
            # Only generate QoS code if explicitly specified
            qos_code = None
            if "qos" in client:
                qos_code = generate_qos_code(client["qos"])

            service_clients.append(
                {
                    "name": client["name"],
                    "service_type": ros_type_to_cpp(client["type"]),
                    "field_name": service_to_field_name(client["name"]),
                    "qos_code": qos_code,
                }
            )
    return service_clients


def prepare_actions(actions_raw: List[Dict[str, Any]]) -> List[Dict[str, str]]:
    """
    Prepare action data for template rendering.
    Converts raw YAML data into template-ready format.
    """
    actions = []
    for action in actions_raw:
        if not action.get("manually_created", False):
            actions.append(
                {
                    "name": action["name"],
                    "action_type": ros_type_to_cpp(action["type"]),
                    "field_name": action_to_field_name(action["name"]),
                }
            )
    return actions


def prepare_action_clients(action_clients_raw: List[Dict[str, Any]]) -> List[Dict[str, str]]:
    """
    Prepare action client data for template rendering.
    Converts raw YAML data into template-ready format.
    """
    action_clients = []
    for client in action_clients_raw:
        if not client.get("manually_created", False):
            action_clients.append(
                {
                    "name": client["name"],
                    "action_type": ros_type_to_cpp(client["type"]),
                    "field_name": action_to_field_name(client["name"]),
                }
            )
    return action_clients


def ros_type_to_python_import(ros_type: str) -> str:
    """
    Convert ROS message type to Python import statement.
    Example: std_msgs/msg/String -> "from std_msgs.msg import String"
    """
    parts = ros_type.split("/")
    if len(parts) >= 3:
        package = parts[0]
        msg_type = parts[1]  # 'msg', 'srv', or 'action'
        class_name = parts[2]
        return f"from {package}.{msg_type} import {class_name}"
    return ""


def ros_type_to_python_class(ros_type: str) -> str:
    """
    Extract just the class name from ROS type.
    Example: std_msgs/msg/String -> "String"
    """
    parts = ros_type.split("/")
    if len(parts) >= 3:
        return parts[2]
    return ""


def generate_python_qos_code(qos_spec: Any) -> tuple[str, Set[str]]:
    """
    Generate Python QoS code from YAML QoS specification.
    Returns tuple of (qos_code, required_imports)

    Supports:
    - Integer: 10 -> ("10", set())
    - String (predefined profile): "SensorDataQoS" -> ("qos_profile_sensor_data", {"qos_profile_sensor_data"})
    - Dict (custom parameters): {reliability: reliable, depth: 10} ->
        ("QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)",
         {"QoSProfile", "ReliabilityPolicy"})
    """
    required_imports = set()

    # Backward compatible: integer
    if isinstance(qos_spec, int):
        return (str(qos_spec), required_imports)

    # Predefined profile: string
    if isinstance(qos_spec, str):
        # Map C++ profile names to Python equivalents
        profile_map = {
            "SensorDataQoS": "qos_profile_sensor_data",
            "SystemDefaultsQoS": "qos_profile_system_default",
            "ServicesQoS": "qos_profile_services_default",
            "ParametersQoS": "qos_profile_parameters",
            "ParameterEventsQoS": "qos_profile_parameter_events",
        }
        python_profile = profile_map.get(qos_spec, qos_spec)
        required_imports.add(python_profile)
        return (python_profile, required_imports)

    # Custom parameters: dict
    if isinstance(qos_spec, dict):
        # Check if it's only a profile without overrides
        if "profile" in qos_spec and len(qos_spec) == 1:
            # Just a profile, treat as string
            profile_map = {
                "SensorDataQoS": "qos_profile_sensor_data",
                "SystemDefaultsQoS": "qos_profile_system_default",
                "ServicesQoS": "qos_profile_services_default",
                "ParametersQoS": "qos_profile_parameters",
                "ParameterEventsQoS": "qos_profile_parameter_events",
            }
            python_profile = profile_map.get(qos_spec["profile"], qos_spec["profile"])
            required_imports.add(python_profile)
            return (python_profile, required_imports)

        required_imports.add("QoSProfile")

        # Build constructor arguments
        args = []

        # Depth
        if "depth" in qos_spec:
            args.append(f"depth={qos_spec['depth']}")
        elif "profile" not in qos_spec:
            args.append("depth=10")

        # Reliability
        if "reliability" in qos_spec:
            required_imports.add("ReliabilityPolicy")
            if qos_spec["reliability"] == "reliable":
                args.append("reliability=ReliabilityPolicy.RELIABLE")
            elif qos_spec["reliability"] == "best_effort":
                args.append("reliability=ReliabilityPolicy.BEST_EFFORT")

        # Durability
        if "durability" in qos_spec:
            required_imports.add("DurabilityPolicy")
            if qos_spec["durability"] == "volatile":
                args.append("durability=DurabilityPolicy.VOLATILE")
            elif qos_spec["durability"] == "transient_local":
                args.append("durability=DurabilityPolicy.TRANSIENT_LOCAL")

        # History
        if "history" in qos_spec:
            required_imports.add("HistoryPolicy")
            if qos_spec["history"] == "keep_last":
                args.append("history=HistoryPolicy.KEEP_LAST")
            elif qos_spec["history"] == "keep_all":
                args.append("history=HistoryPolicy.KEEP_ALL")

        # Liveliness
        if "liveliness" in qos_spec:
            required_imports.add("LivelinessPolicy")
            if qos_spec["liveliness"] == "automatic":
                args.append("liveliness=LivelinessPolicy.AUTOMATIC")
            elif qos_spec["liveliness"] == "manual_by_topic":
                args.append("liveliness=LivelinessPolicy.MANUAL_BY_TOPIC")

        # Deadline and lifespan (Duration)
        if "deadline" in qos_spec or "lifespan" in qos_spec:
            required_imports.add("Duration")

            if "deadline" in qos_spec:
                deadline = qos_spec["deadline"]
                if isinstance(deadline, dict):
                    sec = deadline.get("sec", 0)
                    nsec = deadline.get("nsec", 0)
                    args.append(f"deadline=Duration(seconds={sec}, nanoseconds={nsec})")

            if "lifespan" in qos_spec:
                lifespan = qos_spec["lifespan"]
                if isinstance(lifespan, dict):
                    sec = lifespan.get("sec", 0)
                    nsec = lifespan.get("nsec", 0)
                    args.append(f"lifespan=Duration(seconds={sec}, nanoseconds={nsec})")

        qos_code = f"QoSProfile({', '.join(args)})"
        return (qos_code, required_imports)

    # Default fallback
    return ("10", required_imports)


def prepare_python_publishers(publishers_raw: List[Dict[str, Any]]) -> tuple[List[Dict[str, str]], Set[str]]:
    """
    Prepare publisher data for Python template rendering.
    Returns tuple of (publishers_list, qos_imports_needed)
    """
    publishers = []
    all_qos_imports = set()

    for pub in publishers_raw:
        if not pub.get("manually_created", False):
            qos_code, qos_imports = generate_python_qos_code(pub.get("qos", 10))
            all_qos_imports.update(qos_imports)

            publishers.append(
                {
                    "topic": pub["topic"],
                    "msg_type": pub["type"],
                    "msg_class": ros_type_to_python_class(pub["type"]),
                    "field_name": topic_to_field_name(pub["topic"]),
                    "qos_code": qos_code,
                    "import_stmt": ros_type_to_python_import(pub["type"]),
                }
            )
    return publishers, all_qos_imports


def prepare_python_subscribers(subscribers_raw: List[Dict[str, Any]]) -> tuple[List[Dict[str, str]], Set[str]]:
    """
    Prepare subscriber data for Python template rendering.
    Returns tuple of (subscribers_list, qos_imports_needed)
    """
    subscribers = []
    all_qos_imports = set()

    for sub in subscribers_raw:
        if not sub.get("manually_created", False):
            qos_code, qos_imports = generate_python_qos_code(sub.get("qos", 10))
            all_qos_imports.update(qos_imports)

            subscribers.append(
                {
                    "topic": sub["topic"],
                    "msg_type": sub["type"],
                    "msg_class": ros_type_to_python_class(sub["type"]),
                    "field_name": topic_to_field_name(sub["topic"]),
                    "qos_code": qos_code,
                    "import_stmt": ros_type_to_python_import(sub["type"]),
                }
            )
    return subscribers, all_qos_imports


def collect_includes(
    publishers: List[Dict[str, Any]],
    subscribers: List[Dict[str, Any]],
    services: List[Dict[str, Any]],
    service_clients: List[Dict[str, Any]],
    actions: List[Dict[str, Any]],
    action_clients: List[Dict[str, Any]],
) -> List[str]:
    """Collect all required message, service, and action includes."""
    includes = set()

    for pub in publishers:
        includes.add(ros_type_to_include(pub["type"]))

    for sub in subscribers:
        includes.add(ros_type_to_include(sub["type"]))

    for srv in services:
        includes.add(ros_type_to_service_include(srv["type"]))

    for client in service_clients:
        includes.add(ros_type_to_service_include(client["type"]))

    for action in actions:
        includes.add(ros_type_to_action_include(action["type"]))

    for action_client in action_clients:
        includes.add(ros_type_to_action_include(action_client["type"]))

    return sorted(includes)


def substitute_template_variables(
    interface_data: Dict[str, Any], package_name: str | None, node_name: str | None
) -> None:
    """
    Substitute template variables (${THIS_PACKAGE}, ${THIS_NODE}) in interface_data in-place.
    Raises SystemExit with error message if template is used but value not provided.
    """
    # Substitute ${THIS_NODE} in node.name
    if interface_data["node"]["name"] == "${THIS_NODE}":
        if node_name:
            interface_data["node"]["name"] = node_name
        else:
            print(
                "Error: interface.yaml uses ${THIS_NODE} but no --node-name argument was provided. "
                "Please provide the node name via --node-name argument.",
                file=sys.stderr,
            )
            sys.exit(1)

    # Substitute ${THIS_PACKAGE} in node.package
    if interface_data["node"].get("package", "") == "${THIS_PACKAGE}":
        if package_name:
            interface_data["node"]["package"] = package_name
        else:
            print(
                "Error: interface.yaml uses ${THIS_PACKAGE} but no --package argument was provided. "
                "Please provide the package name via --package argument.",
                file=sys.stderr,
            )
            sys.exit(1)


def get_namespace(interface_data: Dict[str, Any]) -> str:
    """
    Determine the C++ namespace based on package and node name.
    Assumes template variables have already been substituted.
    """
    node_name = interface_data["node"]["name"]
    package = interface_data["node"].get("package", "")

    if package:
        return f"{package}::{node_name}"
    else:
        return node_name


def generate_header(interface_data: Dict[str, Any]) -> str:
    """Generate the complete C++ header file using Jinja2 template.
    Assumes template variables have already been substituted in interface_data.
    """
    # Set up Jinja2 environment
    template_dir = Path(__file__).parent / "templates"
    env = Environment(loader=FileSystemLoader(template_dir), trim_blocks=False, lstrip_blocks=False)
    template = env.get_template("node_interface.hpp.jinja2")

    # Extract data
    node_name = interface_data["node"]["name"]
    package_name = interface_data["node"].get("package", "")
    publishers_raw = interface_data.get("publishers", [])
    subscribers_raw = interface_data.get("subscribers", [])
    services_raw = interface_data.get("services", [])
    service_clients_raw = interface_data.get("service_clients", [])
    actions_raw = interface_data.get("actions", [])
    action_clients_raw = interface_data.get("action_clients", [])

    # Prepare template data
    publishers = prepare_publishers(publishers_raw)
    subscribers = prepare_subscribers(subscribers_raw)
    services = prepare_services(services_raw)
    service_clients = prepare_service_clients(service_clients_raw)
    actions = prepare_actions(actions_raw)
    action_clients = prepare_action_clients(action_clients_raw)
    message_includes = collect_includes(
        publishers_raw, subscribers_raw, services_raw, service_clients_raw, actions_raw, action_clients_raw
    )
    namespace = get_namespace(interface_data)
    class_name = "".join(word.capitalize() for word in node_name.split("_"))

    # Render template
    return template.render(
        node_name=node_name,
        package_name=package_name,
        class_name=class_name,
        namespace=namespace,
        message_includes=message_includes,
        publishers=publishers,
        subscribers=subscribers,
        services=services,
        service_clients=service_clients,
        actions=actions,
        action_clients=action_clients,
    )


def generate_python_interface(interface_data: Dict[str, Any]) -> str:
    """Generate the complete Python interface file using Jinja2 template.
    Assumes template variables have already been substituted in interface_data.
    """
    # Set up Jinja2 environment
    template_dir = Path(__file__).parent / "templates"
    env = Environment(loader=FileSystemLoader(template_dir), trim_blocks=False, lstrip_blocks=False)
    template = env.get_template("node_interface.py.jinja2")

    # Extract data
    node_name = interface_data["node"]["name"]
    package_name = interface_data["node"].get("package", "")
    publishers_raw = interface_data.get("publishers", [])
    subscribers_raw = interface_data.get("subscribers", [])

    # Prepare template data (Python only supports pub/sub for now)
    publishers, pub_qos_imports = prepare_python_publishers(publishers_raw)
    subscribers, sub_qos_imports = prepare_python_subscribers(subscribers_raw)

    # Collect all QoS imports
    qos_imports = pub_qos_imports | sub_qos_imports

    # Collect unique import statements
    message_imports = set()
    for pub in publishers:
        if pub["import_stmt"]:
            message_imports.add(pub["import_stmt"])
    for sub in subscribers:
        if sub["import_stmt"]:
            message_imports.add(sub["import_stmt"])

    # Convert node_name to context class name
    class_name = "".join(word.capitalize() for word in node_name.split("_"))
    context_class = f"{class_name}Context"

    # Render template
    return template.render(
        node_name=node_name,
        package_name=package_name,
        context_class=context_class,
        message_imports=message_imports,
        qos_imports=qos_imports,
        publishers=publishers,
        subscribers=subscribers,
    )


def generate_parameters_yaml(interface_data: Dict[str, Any], package_name: str, node_name: str) -> str:
    """
    Generate parameters.yaml content from interface.yaml data.
    Always returns valid YAML. If no parameters are defined, generates a dummy parameter
    (generate_parameter_library requires at least one parameter).
    """
    # Format: namespace (package::node_name) with parameters underneath
    namespace = f"{package_name}::{node_name}"

    # Get parameters if they exist
    parameters = interface_data.get("parameters", {})

    # generate_parameter_library requires at least one parameter
    # If no parameters are defined, add a dummy one
    if not parameters:
        parameters = {
            "__cake_dummy": {
                "type": "bool",
                "default_value": True,
                "description": "Dummy parameter (cake generates this when no parameters are defined)",
                "read_only": True,
            }
        }

    # Build the YAML structure
    yaml_dict = {namespace: parameters}

    # Convert to YAML string
    return yaml.dump(yaml_dict, default_flow_style=False, sort_keys=False)


def generate_parameters_module(interface_data: Dict[str, Any]) -> str:
    """
    Generate _parameters.py Python module using generate_parameter_library_py.
    Uses static 'parameters' namespace for consistency across all nodes.
    """
    try:
        from generate_parameter_library_py.parse_yaml import GenerateCode
    except ImportError:
        print(
            "Error: generate_parameter_library_py not found. "
            "Install it with: pip install generate_parameter_library",
            file=sys.stderr,
        )
        sys.exit(1)

    # Get parameters if they exist
    parameters = interface_data.get("parameters", {})

    # generate_parameter_library requires at least one parameter
    if not parameters:
        parameters = {
            "__cake_dummy": {
                "type": "bool",
                "default_value": True,
                "description": "Dummy parameter",
                "read_only": True,
            }
        }

    # Static namespace - all nodes use "parameters"
    yaml_dict = {"parameters": parameters}

    # Use TemporaryDirectory for automatic cleanup
    with tempfile.TemporaryDirectory() as tmpdir:
        yaml_path = Path(tmpdir) / "params.yaml"
        with open(yaml_path, "w") as f:
            yaml.dump(yaml_dict, f, default_flow_style=False)

        gen = GenerateCode("python")
        gen.parse(str(yaml_path), "")
        return str(gen)


def generate_init_module(node_name: str) -> str:
    """
    Generate __init__.py that re-exports from _interface.py and _parameters.py.
    """
    # Convert node_name to class name (PascalCase)
    class_name = "".join(word.capitalize() for word in node_name.split("_"))
    context_class = f"{class_name}Context"

    return f"""from ._interface import {context_class}, run
from ._parameters import parameters

Params = parameters.Params
ParamListener = parameters.ParamListener

__all__ = ["{context_class}", "run", "Params", "ParamListener"]
"""


def main():
    parser = argparse.ArgumentParser(description="Generate cake node interface from YAML")
    parser.add_argument("interface_yaml", help="Path to interface.yaml file")
    parser.add_argument("--language", choices=["cpp", "python"], default="cpp", help="Target language")
    parser.add_argument("--package", help="Package name to substitute for ${THIS_PACKAGE}", required=True)
    parser.add_argument("--node-name", help="Node name to substitute for ${THIS_NODE}", required=True)
    parser.add_argument("--output-file", help="Output file path (for C++ header)")
    parser.add_argument("--output-dir", help="Output directory path (for Python module)")

    args = parser.parse_args()

    # Validate output arguments based on language
    if args.language == "cpp" and not args.output_file:
        print("Error: --output-file is required for C++ generation", file=sys.stderr)
        sys.exit(1)
    if args.language == "python" and not args.output_dir:
        print("Error: --output-dir is required for Python generation", file=sys.stderr)
        sys.exit(1)

    # Read and parse YAML
    interface_path = Path(args.interface_yaml)
    if not interface_path.exists():
        print(f"Error: Interface file not found: {interface_path}", file=sys.stderr)
        sys.exit(1)

    with open(interface_path, "r") as f:
        interface_data = yaml.safe_load(f)

    if "node" not in interface_data or "name" not in interface_data["node"]:
        print("Error: interface.yaml must contain 'node.name'", file=sys.stderr)
        sys.exit(1)

    # Substitute all template variables (${THIS_NODE}, ${THIS_PACKAGE}) in-place
    substitute_template_variables(interface_data, args.package, args.node_name)

    node_name = interface_data["node"]["name"]
    package_name = interface_data["node"].get("package", "")

    if args.language == "cpp":
        # Generate C++ header
        header_content = generate_header(interface_data)

        # Ensure output directory exists
        output_file = Path(args.output_file)
        output_file.parent.mkdir(parents=True, exist_ok=True)

        # Write output file
        with open(output_file, "w") as f:
            f.write(header_content)

        print(f"Generated: {output_file}")

        # Always generate parameters.yaml (even if empty)
        if package_name:
            params_content = generate_parameters_yaml(interface_data, package_name, node_name)
            # Generate parameters file alongside the header with .params.yaml extension
            params_file = output_file.parent / f"{output_file.stem}.params.yaml"
            with open(params_file, "w") as f:
                f.write(params_content)
            print(f"Generated: {params_file}")

    elif args.language == "python":
        # Generate Python module files
        output_dir = Path(args.output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)

        # Generate _interface.py
        interface_content = generate_python_interface(interface_data)
        interface_file = output_dir / "_interface.py"
        with open(interface_file, "w") as f:
            f.write(interface_content)
        print(f"Generated: {interface_file}")

        # Generate _parameters.py
        parameters_content = generate_parameters_module(interface_data)
        parameters_file = output_dir / "_parameters.py"
        with open(parameters_file, "w") as f:
            f.write(parameters_content)
        print(f"Generated: {parameters_file}")

        # Generate __init__.py
        init_content = generate_init_module(node_name)
        init_file = output_dir / "__init__.py"
        with open(init_file, "w") as f:
            f.write(init_content)
        print(f"Generated: {init_file}")


if __name__ == "__main__":
    main()
