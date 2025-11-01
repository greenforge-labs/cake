#!/usr/bin/env python3

"""
Code generator for cake ROS2 node interfaces.
Parses interface.yaml and generates C++ header with publishers, subscribers, context, and base node class.
"""

import argparse
import sys
from pathlib import Path
from typing import Dict, List, Any
import yaml
from jinja2 import Environment, FileSystemLoader


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
    """
    parts = ros_type.split("/")
    if len(parts) >= 3:
        # Convert last part (message name) to lowercase
        parts[-1] = parts[-1].lower()
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
            publishers.append({
                'topic': pub['topic'],
                'msg_type': ros_type_to_cpp(pub['type']),
                'field_name': topic_to_field_name(pub['topic']),
                'qos_code': generate_qos_code(pub.get('qos', 10))
            })
    return publishers


def prepare_subscribers(subscribers_raw: List[Dict[str, Any]]) -> List[Dict[str, str]]:
    """
    Prepare subscriber data for template rendering.
    Converts raw YAML data into template-ready format.
    """
    subscribers = []
    for sub in subscribers_raw:
        if not sub.get("manually_created", False):
            subscribers.append({
                'topic': sub['topic'],
                'msg_type': ros_type_to_cpp(sub['type']),
                'field_name': topic_to_field_name(sub['topic']),
                'qos_code': generate_qos_code(sub.get('qos', 10))
            })
    return subscribers


def collect_includes(publishers: List[Dict[str, Any]], subscribers: List[Dict[str, Any]]) -> List[str]:
    """Collect all required message includes."""
    includes = set()

    for pub in publishers:
        includes.add(ros_type_to_include(pub['type']))

    for sub in subscribers:
        includes.add(ros_type_to_include(sub['type']))

    return sorted(includes)


def get_namespace(interface_data: Dict[str, Any], package_name: str = None) -> str:
    """
    Determine the C++ namespace based on package and node name.
    Handles ${THIS_PACKAGE} substitution.
    """
    node_name = interface_data["node"]["name"]
    package = interface_data["node"].get("package", "")

    # Substitute ${THIS_PACKAGE} with actual package name if provided
    if package == "${THIS_PACKAGE}":
        if package_name:
            package = package_name
        else:
            # Error: ${THIS_PACKAGE} used but no package name provided
            raise ValueError(
                "interface.yaml uses ${THIS_PACKAGE} but no --package argument was provided. "
                "Please provide the package name via --package argument."
            )

    if package:
        return f"{package}::{node_name}"
    else:
        return node_name


def generate_header(interface_data: Dict[str, Any], package_name: str = None) -> str:
    """Generate the complete C++ header file using Jinja2 template."""
    # Set up Jinja2 environment
    template_dir = Path(__file__).parent / "templates"
    env = Environment(
        loader=FileSystemLoader(template_dir),
        trim_blocks=False,
        lstrip_blocks=False
    )
    template = env.get_template("node_interface.hpp.jinja2")

    # Extract data
    node_name = interface_data["node"]["name"]
    publishers_raw = interface_data.get("publishers", [])
    subscribers_raw = interface_data.get("subscribers", [])

    # Prepare template data
    publishers = prepare_publishers(publishers_raw)
    subscribers = prepare_subscribers(subscribers_raw)
    message_includes = collect_includes(publishers_raw, subscribers_raw)
    namespace = get_namespace(interface_data, package_name)
    class_name = "".join(word.capitalize() for word in node_name.split("_"))

    # Render template
    return template.render(
        node_name=node_name,
        class_name=class_name,
        namespace=namespace,
        message_includes=message_includes,
        publishers=publishers,
        subscribers=subscribers
    )


def main():
    parser = argparse.ArgumentParser(
        description="Generate cake node interface header from YAML"
    )
    parser.add_argument("output_file", help="Output header file path")
    parser.add_argument("interface_yaml", help="Path to interface.yaml file")
    parser.add_argument("--package", help="Package name to substitute for ${THIS_PACKAGE}", default=None)

    args = parser.parse_args()

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

    # Generate header content
    header_content = generate_header(interface_data, args.package)

    # Ensure output directory exists
    output_file = Path(args.output_file)
    output_file.parent.mkdir(parents=True, exist_ok=True)

    # Write output file
    with open(output_file, "w") as f:
        f.write(header_content)

    print(f"Generated: {output_file}")


if __name__ == "__main__":
    main()
