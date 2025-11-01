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


def generate_publishers_struct(publishers: List[Dict[str, Any]], namespace: str) -> str:
    """Generate the Publishers struct template."""
    if not publishers:
        return f"""template <typename ContextType> struct {namespace}Publishers {{}};
"""

    lines = [f"template <typename ContextType> struct {namespace}Publishers {{"]
    for pub in publishers:
        if pub.get("manually_created", False):
            continue
        topic_name = pub["topic"]
        msg_type = ros_type_to_cpp(pub["type"])
        # Convert topic name to valid C++ identifier (replace / with _)
        field_name = topic_name.replace("/", "_").lstrip("_")
        lines.append(f"    rclcpp::Publisher<{msg_type}>::SharedPtr {field_name};")
    lines.append("};")
    lines.append("")
    return "\n".join(lines)


def generate_subscribers_struct(
    subscribers: List[Dict[str, Any]], namespace: str
) -> str:
    """Generate the Subscribers struct template."""
    if not subscribers:
        return f"""template <typename ContextType> struct {namespace}Subscribers {{}};
"""

    lines = [f"template <typename ContextType> struct {namespace}Subscribers {{"]
    for sub in subscribers:
        if sub.get("manually_created", False):
            continue
        topic_name = sub["topic"]
        msg_type = ros_type_to_cpp(sub["type"])
        # Convert topic name to valid C++ identifier
        field_name = topic_name.replace("/", "_").lstrip("_")
        lines.append(
            f"    std::shared_ptr<cake::Subscriber<{msg_type}, ContextType>> {field_name};"
        )
    lines.append("};")
    lines.append("")
    return "\n".join(lines)


def generate_context_struct(node_name: str, namespace: str) -> str:
    """Generate the Context struct template."""
    return f"""template <typename DerivedContextType> struct {namespace}Context : cake::Context {{
    {namespace}Publishers<DerivedContextType> publishers;
    {namespace}Subscribers<DerivedContextType> subscribers;
}};
"""


def generate_publisher_init(publishers: List[Dict[str, Any]]) -> str:
    """Generate publisher initialization code."""
    if not publishers:
        return ""

    lines = ["        // init publishers"]
    for pub in publishers:
        if pub.get("manually_created", False):
            continue
        topic_name = pub["topic"]
        msg_type = ros_type_to_cpp(pub["type"])
        field_name = topic_name.replace("/", "_").lstrip("_")
        qos = pub.get("qos", 10)
        lines.append(
            f'        ctx->publishers.{field_name} = ctx->node->template create_publisher<{msg_type}>("{topic_name}", {qos});'
        )
    lines.append("")
    return "\n".join(lines)


def generate_subscriber_init(subscribers: List[Dict[str, Any]]) -> str:
    """Generate subscriber initialization code."""
    if not subscribers:
        return ""

    lines = ["        // init subscribers"]
    for sub in subscribers:
        if sub.get("manually_created", False):
            continue
        topic_name = sub["topic"]
        msg_type = ros_type_to_cpp(sub["type"])
        field_name = topic_name.replace("/", "_").lstrip("_")
        qos = sub.get("qos", 10)
        lines.append(
            f'        ctx->subscribers.{field_name} = cake::create_subscriber<{msg_type}>(ctx, "{topic_name}", {qos});'
        )
    lines.append("")
    return "\n".join(lines)


def generate_base_node_class(
    node_name: str,
    namespace: str,
    publishers: List[Dict[str, Any]],
    subscribers: List[Dict[str, Any]],
) -> str:
    """Generate the base node class template."""
    pub_init = generate_publisher_init(publishers)
    sub_init = generate_subscriber_init(subscribers)

    return f"""template <
    typename ContextType,
    auto init_func,
    auto extend_options = [](rclcpp::NodeOptions options) {{ return options; }}>
class {namespace}Base : public cake::BaseNode<"{node_name}", extend_options> {{
  public:
    explicit {namespace}Base(const rclcpp::NodeOptions &options) : cake::BaseNode<"{node_name}", extend_options>(options) {{
        static_assert(
            std::is_base_of_v<{namespace}Context<ContextType>, ContextType>, "ContextType must be a child of {namespace}Context"
        );

        // init context
        auto ctx = std::make_shared<ContextType>();
        ctx->node = this->node_;

{pub_init}{sub_init}        // TODO init services and actions

        init_func(ctx);
    }}
}};
"""


def collect_includes(
    publishers: List[Dict[str, Any]], subscribers: List[Dict[str, Any]]
) -> List[str]:
    """Collect all required message includes."""
    includes = set()

    for pub in publishers:
        if not pub.get("manually_created", False):
            includes.add(ros_type_to_include(pub["type"]))

    for sub in subscribers:
        if not sub.get("manually_created", False):
            includes.add(ros_type_to_include(sub["type"]))

    return sorted(includes)


def generate_header(interface_data: Dict[str, Any]) -> str:
    """Generate the complete C++ header file."""
    node_info = interface_data["node"]
    node_name = node_info["name"]
    package = node_info.get("package", "")

    # Get publishers and subscribers
    publishers = interface_data.get("publishers", [])
    subscribers = interface_data.get("subscribers", [])

    # Collect includes
    message_includes = collect_includes(publishers, subscribers)

    # Convert node_name to CapitalCase for struct/class names
    namespace = "".join(word.capitalize() for word in node_name.split("_"))

    # Generate header
    lines = ["#pragma once", ""]
    lines.append("#include <memory>")
    lines.append("#include <rclcpp/rclcpp.hpp>")
    lines.append("")

    # Add message includes
    for include in message_includes:
        lines.append(f"#include <{include}>")
    if message_includes:
        lines.append("")

    # Add cake includes
    lines.append("#include <cake/base_node.hpp>")
    lines.append("#include <cake/context.hpp>")
    if subscribers:
        lines.append("#include <cake/subscriber.hpp>")
    lines.append("")

    # Determine namespace
    if package and package != "${THIS_PACKAGE}":
        ns = f"{package}::{node_name}"
    else:
        # For ${THIS_PACKAGE}, we can't know the package name, so just use node_name
        # The user will need to wrap this in their own namespace
        ns = node_name

    # Open namespace
    lines.append(f"namespace {ns} {{")
    lines.append("")

    # Generate structures
    lines.append(generate_publishers_struct(publishers, namespace))
    lines.append(generate_subscribers_struct(subscribers, namespace))
    lines.append(generate_context_struct(node_name, namespace))
    lines.append("")
    lines.append(
        generate_base_node_class(node_name, namespace, publishers, subscribers)
    )

    # Close namespace
    lines.append(f"}} // namespace {ns}")
    lines.append("")

    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(
        description="Generate cake node interface header from YAML"
    )
    parser.add_argument("output_file", help="Output header file path")
    parser.add_argument("interface_yaml", help="Path to interface.yaml file")

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
    header_content = generate_header(interface_data)

    # Ensure output directory exists
    output_file = Path(args.output_file)
    output_file.parent.mkdir(parents=True, exist_ok=True)

    # Write output file
    with open(output_file, "w") as f:
        f.write(header_content)

    print(f"Generated: {output_file}")


if __name__ == "__main__":
    main()
