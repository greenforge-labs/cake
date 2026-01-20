#!/usr/bin/env python3

"""
Code generator for cake ROS2 node interfaces.
Parses interface.yaml and generates C++ header with publishers, subscribers, context, and base node class.
"""

import argparse
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
import re
import sys
import tempfile

from ament_index_python.packages import get_package_share_directory
from jinja2 import Environment, FileSystemLoader
from jsonschema import Draft202012Validator, RefResolver
import yaml

from typing import Any, Dict, List, Set


class EntityKind(Enum):
    """Entity types that can be defined in interface.yaml."""

    PUBLISHER = "publisher"
    SUBSCRIBER = "subscriber"
    SERVICE = "service"
    SERVICE_CLIENT = "service_client"
    ACTION = "action"
    ACTION_CLIENT = "action_client"


@dataclass(frozen=True)
class EntityConfig:
    """Configuration for preparing entity data for template rendering."""

    kind: EntityKind
    name_key: str  # Key in YAML: "topic" for pub/sub, "name" for others
    output_type_key: str  # Key for template: "msg_type", "service_type", or "action_type"
    has_qos: bool  # Whether QoS configuration is applicable


# Constants
DUMMY_PARAM_NAME = "__cake_dummy"

# Entity configurations mapping schema entity types to their template requirements
ENTITY_CONFIGS: Dict[EntityKind, EntityConfig] = {
    EntityKind.PUBLISHER: EntityConfig(
        kind=EntityKind.PUBLISHER,
        name_key="topic",
        output_type_key="msg_type",
        has_qos=True,
    ),
    EntityKind.SUBSCRIBER: EntityConfig(
        kind=EntityKind.SUBSCRIBER,
        name_key="topic",
        output_type_key="msg_type",
        has_qos=True,
    ),
    EntityKind.SERVICE: EntityConfig(
        kind=EntityKind.SERVICE,
        name_key="name",
        output_type_key="service_type",
        has_qos=False,
    ),
    EntityKind.SERVICE_CLIENT: EntityConfig(
        kind=EntityKind.SERVICE_CLIENT,
        name_key="name",
        output_type_key="service_type",
        has_qos=False,
    ),
    EntityKind.ACTION: EntityConfig(
        kind=EntityKind.ACTION,
        name_key="name",
        output_type_key="action_type",
        has_qos=False,
    ),
    EntityKind.ACTION_CLIENT: EntityConfig(
        kind=EntityKind.ACTION_CLIENT,
        name_key="name",
        output_type_key="action_type",
        has_qos=False,
    ),
}

# Regular expression for parameter reference pattern ${param:parameter_name}
PARAM_REF_PATTERN = re.compile(r"^\$\{param:([a-zA-Z_][a-zA-Z0-9_]*)\}$")

# QoS field type requirements for parameter validation
QOS_FIELD_TYPES = {
    "history": "int",
    "reliability": "string",
    "durability": "string",
    "liveliness": "string",
    "deadline_ms": "int",
    "lifespan_ms": "int",
    "lease_duration_ms": "int",
}


# Custom exception for validation errors
class InterfaceValidationError(Exception):
    """Raised when interface.yaml validation fails."""

    pass


def is_param_ref(value: Any) -> bool:
    """Check if a value is a parameter reference (${param:name} format)."""
    if not isinstance(value, str):
        return False
    return PARAM_REF_PATTERN.match(value) is not None


def extract_param_name(ref: str) -> str:
    """Extract the parameter name from a ${param:name} reference."""
    match = PARAM_REF_PATTERN.match(ref)
    if not match:
        raise ValueError(f"Invalid parameter reference format: {ref}")
    return match.group(1)


def validate_param_references(interface_data: Dict[str, Any]) -> None:
    """
    Validate parameter references in QoS fields.

    Checks:
    1. Referenced parameter exists in parameters section
    2. Referenced parameter has read_only: true
    3. Parameter type is compatible with the QoS field

    Raises:
        InterfaceValidationError: If validation fails
    """
    parameters = interface_data.get("parameters", {})

    def validate_qos_param_ref(qos_spec: Dict[str, Any], entity_type: str, entity_name: str) -> None:
        """Validate parameter references in a single QoS specification."""
        for field_name, value in qos_spec.items():
            if not is_param_ref(value):
                continue

            param_name = extract_param_name(value)

            # Check 1: Parameter exists
            if param_name not in parameters:
                raise InterfaceValidationError(
                    f"{entity_type} '{entity_name}' qos.{field_name}: "
                    f"references non-existent parameter '{param_name}'"
                )

            param_def = parameters[param_name]

            # Check 2: Parameter is read_only
            if not param_def.get("read_only", False):
                raise InterfaceValidationError(
                    f"{entity_type} '{entity_name}' qos.{field_name}: "
                    f"parameter '{param_name}' must have read_only: true to be used in QoS configuration"
                )

            # Check 3: Type compatibility
            expected_type = QOS_FIELD_TYPES.get(field_name)
            if expected_type:
                actual_type = param_def.get("type", "")
                if actual_type != expected_type:
                    raise InterfaceValidationError(
                        f"{entity_type} '{entity_name}' qos.{field_name}: "
                        f"parameter '{param_name}' has type '{actual_type}', but '{expected_type}' is required"
                    )

    # Validate publishers
    for pub in interface_data.get("publishers", []):
        if pub.get("manually_created", False):
            continue
        if "qos" in pub:
            validate_qos_param_ref(pub["qos"], "publisher", pub["topic"])

    # Validate subscribers
    for sub in interface_data.get("subscribers", []):
        if sub.get("manually_created", False):
            continue
        if "qos" in sub:
            validate_qos_param_ref(sub["qos"], "subscriber", sub["topic"])


def load_schemas() -> tuple[dict, dict]:
    """Load interface and parameter schemas from the installed package share directory."""
    share_dir = get_package_share_directory("cake")
    schema_dir = Path(share_dir) / "schemas"

    with open(schema_dir / "interface.schema.yaml") as f:
        interface_schema = yaml.safe_load(f)

    with open(schema_dir / "parameter.schema.yaml") as f:
        parameter_schema = yaml.safe_load(f)

    return interface_schema, parameter_schema


def validate_with_schema(interface_data: Dict[str, Any]) -> None:
    """
    Validate interface data against the YAML schema.

    Raises:
        InterfaceValidationError: If validation fails
    """
    interface_schema, parameter_schema = load_schemas()

    # Create a resolver that knows about the parameter schema (for $ref resolution)
    store = {"parameter.schema.yaml": parameter_schema}
    resolver = RefResolver.from_schema(interface_schema, store=store)

    validator = Draft202012Validator(interface_schema, resolver=resolver)
    errors = list(validator.iter_errors(interface_data))

    if errors:
        messages = []
        for error in errors:
            # Build readable path
            path = " -> ".join(str(p) for p in error.path) if error.path else "(root)"
            messages.append(f"  [{path}] {error.message}")

        raise InterfaceValidationError(
            f"Interface validation failed with {len(errors)} error(s):\n" + "\n".join(messages)
        )


def validate_interface_yaml(interface_data: Dict[str, Any]) -> None:
    """
    Validate the complete interface.yaml structure using schema.

    Raises:
        InterfaceValidationError: If validation fails
    """
    validate_with_schema(interface_data)
    validate_param_references(interface_data)


def get_dummy_parameter() -> Dict[str, Any]:
    """
    Create a dummy parameter for generate_parameter_library.
    Required when no parameters are defined (library needs at least one parameter).
    """
    return {
        DUMMY_PARAM_NAME: {
            "type": "bool",
            "default_value": True,
            "description": "Dummy parameter (cake generates this when no parameters are defined)",
            "read_only": True,
        }
    }


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


def name_to_field_name(name: str) -> str:
    """
    Convert topic/service/action name to valid C++ identifier.
    Example: /cmd_vel -> cmd_vel, /robot/status -> robot_status
    """
    return name.replace("/", "_").lstrip("_")


def generate_qos_code(qos_spec: Dict[str, Any]) -> tuple[str, bool]:
    """
    Generate C++ QoS code from YAML QoS specification.

    New format:
    - history: integer > 0 for KEEP_LAST(n), or "ALL" for KEEP_ALL, or ${param:name} (required)
    - reliability: "BEST_EFFORT" or "RELIABLE", or ${param:name} (required)
    - durability: "TRANSIENT_LOCAL" or "VOLATILE", or ${param:name} (optional)
    - deadline_ms: milliseconds, or ${param:name} (optional)
    - lifespan_ms: milliseconds, or ${param:name} (optional)
    - liveliness: "AUTOMATIC" or "MANUAL_BY_TOPIC", or ${param:name} (optional)
    - lease_duration_ms: milliseconds, or ${param:name} (optional)

    Returns:
        tuple of (qos_code, needs_qos_helpers) where needs_qos_helpers is True if
        the code uses cake::to_* helper functions for enum conversion.
    """
    needs_qos_helpers = False

    # Handle history - required field
    history = qos_spec["history"]
    if is_param_ref(history):
        param_name = extract_param_name(history)
        base_qos = f"rclcpp::QoS(ctx->params.{param_name})"
    elif history == "ALL":
        base_qos = "rclcpp::QoS(rclcpp::KeepAll())"
    else:
        base_qos = f"rclcpp::QoS({history})"

    # Build chain of method calls
    methods = []

    # Reliability - required field
    reliability = qos_spec["reliability"]
    if is_param_ref(reliability):
        param_name = extract_param_name(reliability)
        methods.append(f".reliability(cake::to_reliability(ctx->params.{param_name}))")
        needs_qos_helpers = True
    elif reliability == "RELIABLE":
        methods.append(".reliable()")
    elif reliability == "BEST_EFFORT":
        methods.append(".best_effort()")

    # Durability - optional
    if "durability" in qos_spec:
        durability = qos_spec["durability"]
        if is_param_ref(durability):
            param_name = extract_param_name(durability)
            methods.append(f".durability(cake::to_durability(ctx->params.{param_name}))")
            needs_qos_helpers = True
        elif durability == "VOLATILE":
            methods.append(".durability_volatile()")
        elif durability == "TRANSIENT_LOCAL":
            methods.append(".transient_local()")

    # Deadline - optional (convert ms to nanoseconds)
    if "deadline_ms" in qos_spec:
        deadline_ms = qos_spec["deadline_ms"]
        if is_param_ref(deadline_ms):
            param_name = extract_param_name(deadline_ms)
            methods.append(f".deadline(rclcpp::Duration::from_nanoseconds(ctx->params.{param_name} * 1000000LL))")
        else:
            ns = deadline_ms * 1_000_000
            methods.append(f".deadline(rclcpp::Duration::from_nanoseconds({ns}))")

    # Lifespan - optional (convert ms to nanoseconds)
    if "lifespan_ms" in qos_spec:
        lifespan_ms = qos_spec["lifespan_ms"]
        if is_param_ref(lifespan_ms):
            param_name = extract_param_name(lifespan_ms)
            methods.append(f".lifespan(rclcpp::Duration::from_nanoseconds(ctx->params.{param_name} * 1000000LL))")
        else:
            ns = lifespan_ms * 1_000_000
            methods.append(f".lifespan(rclcpp::Duration::from_nanoseconds({ns}))")

    # Liveliness - optional
    if "liveliness" in qos_spec:
        liveliness = qos_spec["liveliness"]
        if is_param_ref(liveliness):
            param_name = extract_param_name(liveliness)
            methods.append(f".liveliness(cake::to_liveliness(ctx->params.{param_name}))")
            needs_qos_helpers = True
        elif liveliness == "AUTOMATIC":
            methods.append(".liveliness(rclcpp::LivelinessPolicy::Automatic)")
        elif liveliness == "MANUAL_BY_TOPIC":
            methods.append(".liveliness(rclcpp::LivelinessPolicy::ManualByTopic)")

    # Liveliness lease duration - optional (convert ms to nanoseconds)
    if "lease_duration_ms" in qos_spec:
        lease_duration_ms = qos_spec["lease_duration_ms"]
        if is_param_ref(lease_duration_ms):
            param_name = extract_param_name(lease_duration_ms)
            methods.append(
                f".liveliness_lease_duration(rclcpp::Duration::from_nanoseconds(ctx->params.{param_name} * 1000000LL))"
            )
        else:
            ns = lease_duration_ms * 1_000_000
            methods.append(f".liveliness_lease_duration(rclcpp::Duration::from_nanoseconds({ns}))")

    return base_qos + "".join(methods), needs_qos_helpers


def prepare_entities(
    entities_raw: List[Dict[str, Any]],
    config: EntityConfig,
) -> tuple[List[Dict[str, str]], bool]:
    """
    Generic entity preparation for C++ template rendering.
    Converts raw YAML data into template-ready format based on entity configuration.

    Returns:
        tuple of (entities_list, needs_qos_helpers) where needs_qos_helpers is True if
        any entity uses QoS parameter references that require cake::to_* helpers.
    """
    entities = []
    needs_qos_helpers = False
    for entity in entities_raw:
        if entity.get("manually_created", False):
            continue

        name_value = entity[config.name_key]
        prepared = {
            config.name_key: name_value,
            config.output_type_key: ros_type_to_cpp(entity["type"]),
            "field_name": name_to_field_name(name_value),
        }

        if config.has_qos:
            # QoS is required for publishers/subscribers in the new format
            qos_code, entity_needs_helpers = generate_qos_code(entity["qos"])
            prepared["qos_code"] = qos_code
            if entity_needs_helpers:
                needs_qos_helpers = True

        entities.append(prepared)
    return entities, needs_qos_helpers


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


def generate_python_qos_code(qos_spec: Dict[str, Any]) -> tuple[str, Set[str], bool]:
    """
    Generate Python QoS code from YAML QoS specification.
    Returns tuple of (qos_code, required_imports, needs_qos_helpers)

    New format:
    - history: integer > 0 for KEEP_LAST(n), or "ALL" for KEEP_ALL, or ${param:name} (required)
    - reliability: "BEST_EFFORT" or "RELIABLE", or ${param:name} (required)
    - durability: "TRANSIENT_LOCAL" or "VOLATILE", or ${param:name} (optional)
    - deadline_ms: milliseconds, or ${param:name} (optional)
    - lifespan_ms: milliseconds, or ${param:name} (optional)
    - liveliness: "AUTOMATIC" or "MANUAL_BY_TOPIC", or ${param:name} (optional)
    - lease_duration_ms: milliseconds, or ${param:name} (optional)
    """
    required_imports: Set[str] = set()
    required_imports.add("QoSProfile")
    required_imports.add("HistoryPolicy")
    required_imports.add("ReliabilityPolicy")
    needs_qos_helpers = False

    # Build constructor arguments
    args = []

    # History - required field
    history = qos_spec["history"]
    if is_param_ref(history):
        param_name = extract_param_name(history)
        args.append("history=HistoryPolicy.KEEP_LAST")
        args.append(f"depth=ctx.params.{param_name}")
    elif history == "ALL":
        args.append("history=HistoryPolicy.KEEP_ALL")
    else:
        args.append("history=HistoryPolicy.KEEP_LAST")
        args.append(f"depth={history}")

    # Reliability - required field
    reliability = qos_spec["reliability"]
    if is_param_ref(reliability):
        param_name = extract_param_name(reliability)
        args.append(f"reliability=_to_reliability(ctx.params.{param_name})")
        needs_qos_helpers = True
    elif reliability == "RELIABLE":
        args.append("reliability=ReliabilityPolicy.RELIABLE")
    elif reliability == "BEST_EFFORT":
        args.append("reliability=ReliabilityPolicy.BEST_EFFORT")

    # Durability - optional
    if "durability" in qos_spec:
        durability = qos_spec["durability"]
        if is_param_ref(durability):
            param_name = extract_param_name(durability)
            args.append(f"durability=_to_durability(ctx.params.{param_name})")
            needs_qos_helpers = True
        else:
            required_imports.add("DurabilityPolicy")
            if durability == "VOLATILE":
                args.append("durability=DurabilityPolicy.VOLATILE")
            elif durability == "TRANSIENT_LOCAL":
                args.append("durability=DurabilityPolicy.TRANSIENT_LOCAL")

    # Deadline - optional (convert ms to nanoseconds)
    if "deadline_ms" in qos_spec:
        required_imports.add("Duration")
        deadline_ms = qos_spec["deadline_ms"]
        if is_param_ref(deadline_ms):
            param_name = extract_param_name(deadline_ms)
            args.append(f"deadline=Duration(nanoseconds=ctx.params.{param_name} * 1000000)")
        else:
            ns = deadline_ms * 1_000_000
            args.append(f"deadline=Duration(nanoseconds={ns})")

    # Lifespan - optional (convert ms to nanoseconds)
    if "lifespan_ms" in qos_spec:
        required_imports.add("Duration")
        lifespan_ms = qos_spec["lifespan_ms"]
        if is_param_ref(lifespan_ms):
            param_name = extract_param_name(lifespan_ms)
            args.append(f"lifespan=Duration(nanoseconds=ctx.params.{param_name} * 1000000)")
        else:
            ns = lifespan_ms * 1_000_000
            args.append(f"lifespan=Duration(nanoseconds={ns})")

    # Liveliness - optional
    if "liveliness" in qos_spec:
        liveliness = qos_spec["liveliness"]
        if is_param_ref(liveliness):
            param_name = extract_param_name(liveliness)
            args.append(f"liveliness=_to_liveliness(ctx.params.{param_name})")
            needs_qos_helpers = True
        else:
            required_imports.add("LivelinessPolicy")
            if liveliness == "AUTOMATIC":
                args.append("liveliness=LivelinessPolicy.AUTOMATIC")
            elif liveliness == "MANUAL_BY_TOPIC":
                args.append("liveliness=LivelinessPolicy.MANUAL_BY_TOPIC")

    # Liveliness lease duration - optional (convert ms to nanoseconds)
    if "lease_duration_ms" in qos_spec:
        required_imports.add("Duration")
        lease_duration_ms = qos_spec["lease_duration_ms"]
        if is_param_ref(lease_duration_ms):
            param_name = extract_param_name(lease_duration_ms)
            args.append(f"liveliness_lease_duration=Duration(nanoseconds=ctx.params.{param_name} * 1000000)")
        else:
            ns = lease_duration_ms * 1_000_000
            args.append(f"liveliness_lease_duration=Duration(nanoseconds={ns})")

    qos_code = f"QoSProfile({', '.join(args)})"
    return (qos_code, required_imports, needs_qos_helpers)


def _get_python_class_key(output_type_key: str) -> str:
    """
    Map output_type_key to the corresponding class key for Python templates.
    msg_type -> msg_class, service_type -> srv_class, action_type -> action_class
    """
    mapping = {
        "msg_type": "msg_class",
        "service_type": "srv_class",
        "action_type": "action_class",
    }
    return mapping.get(output_type_key, output_type_key.replace("_type", "_class"))


def _get_python_type_key(output_type_key: str) -> str:
    """
    Map output_type_key to the corresponding type key for Python templates.
    msg_type -> msg_type, service_type -> srv_type, action_type -> action_type
    """
    mapping = {
        "msg_type": "msg_type",
        "service_type": "srv_type",
        "action_type": "action_type",
    }
    return mapping.get(output_type_key, output_type_key)


def prepare_python_entities(
    entities_raw: List[Dict[str, Any]],
    config: EntityConfig,
) -> tuple[List[Dict[str, str]], Set[str], bool]:
    """
    Generic entity preparation for Python template rendering.
    Converts raw YAML data into template-ready format based on entity configuration.
    Returns tuple of (entities_list, qos_imports_needed, needs_qos_helpers).
    """
    entities = []
    all_qos_imports: Set[str] = set()
    needs_qos_helpers = False

    python_type_key = _get_python_type_key(config.output_type_key)
    python_class_key = _get_python_class_key(config.output_type_key)

    for entity in entities_raw:
        if entity.get("manually_created", False):
            continue

        name_value = entity[config.name_key]
        ros_type = entity["type"]

        prepared: Dict[str, Any] = {
            config.name_key: name_value,
            python_type_key: ros_type,
            python_class_key: ros_type_to_python_class(ros_type),
            "field_name": name_to_field_name(name_value),
            "import_stmt": ros_type_to_python_import(ros_type),
        }

        if config.has_qos:
            # QoS is required for publishers/subscribers in the new format
            qos_code, qos_imports, entity_needs_helpers = generate_python_qos_code(entity["qos"])
            prepared["qos_code"] = qos_code
            all_qos_imports.update(qos_imports)
            if entity_needs_helpers:
                needs_qos_helpers = True

        entities.append(prepared)
    return entities, all_qos_imports, needs_qos_helpers


def collect_includes(interface_data: Dict[str, Any]) -> List[str]:
    """Collect all required message, service, and action includes."""
    includes = set()

    entity_keys = ["publishers", "subscribers", "services", "service_clients", "actions", "action_clients"]
    for key in entity_keys:
        for entity in interface_data.get(key, []):
            includes.add(ros_type_to_include(entity["type"]))

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


def get_implementation_namespace(interface_data: Dict[str, Any]) -> str:
    """
    Determine the C++ implementation namespace (package::node_name).
    This is where the actual node implementation lives to avoid name collisions.
    Assumes template variables have already been substituted.
    """
    node_name = interface_data["node"]["name"]
    package = interface_data["node"].get("package", "")

    if package:
        return f"{package}::{node_name}"
    else:
        return node_name


def get_namespace(interface_data: Dict[str, Any]) -> str:
    """
    Determine the C++ export namespace (just the package name).
    This is the namespace used for component registration.
    Assumes template variables have already been substituted.
    """
    package = interface_data["node"].get("package", "")
    return package if package else ""


def get_class_name(node_name: str) -> str:
    """
    Convert snake_case node name to PascalCase class name.
    Example: "my_node" -> "MyNode"
    """
    return "".join(word.capitalize() for word in node_name.split("_"))


def get_plugin_name(interface_data: Dict[str, Any]) -> str:
    """
    Get the full ROS 2 component plugin name.
    Format: {package}::{NodeNamePascal}
    Example: "test_package::TestNode"
    Assumes template variables have already been substituted in interface_data.
    """
    node_name = interface_data["node"]["name"]
    package = interface_data["node"].get("package", "")
    class_name = get_class_name(node_name)

    if package:
        return f"{package}::{class_name}"
    else:
        return class_name


def generate_interface_yaml(
    interface_yaml_path: Path,
    interface_data: Dict[str, Any],
    package_name: str | None,
    node_name: str | None,
    is_cpp: bool,
) -> str:
    """
    Generate processed interface YAML with token replacement and plugin field (for C++).

    Args:
        interface_yaml_path: Path to the original interface.yaml file
        interface_data: Parsed interface data (with template variables already substituted)
        package_name: Package name for token replacement
        node_name: Node name for token replacement
        is_cpp: True if generating for C++, False for Python

    Returns:
        Processed YAML content as string
    """
    # Read original interface.yaml file
    with open(interface_yaml_path, "r") as f:
        yaml_content = f.read()

    # Perform token replacement on raw text
    if node_name:
        yaml_content = yaml_content.replace("${THIS_NODE}", node_name)
    if package_name:
        yaml_content = yaml_content.replace("${THIS_PACKAGE}", package_name)

    # Parse the YAML for consistent formatting
    yaml_data = yaml.safe_load(yaml_content)

    # For C++ nodes, add plugin field under node section
    if is_cpp:
        plugin_name = get_plugin_name(interface_data)
        yaml_data["node"]["plugin"] = plugin_name

    # Serialize back to YAML for consistent formatting
    yaml_content = yaml.dump(yaml_data, default_flow_style=False, sort_keys=False)

    return yaml_content


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

    # Prepare template data using generic entity preparation
    publishers, pub_needs_helpers = prepare_entities(
        interface_data.get("publishers", []), ENTITY_CONFIGS[EntityKind.PUBLISHER]
    )
    subscribers, sub_needs_helpers = prepare_entities(
        interface_data.get("subscribers", []), ENTITY_CONFIGS[EntityKind.SUBSCRIBER]
    )
    services, _ = prepare_entities(interface_data.get("services", []), ENTITY_CONFIGS[EntityKind.SERVICE])
    service_clients, _ = prepare_entities(
        interface_data.get("service_clients", []), ENTITY_CONFIGS[EntityKind.SERVICE_CLIENT]
    )
    actions, _ = prepare_entities(interface_data.get("actions", []), ENTITY_CONFIGS[EntityKind.ACTION])
    action_clients, _ = prepare_entities(
        interface_data.get("action_clients", []), ENTITY_CONFIGS[EntityKind.ACTION_CLIENT]
    )
    message_includes = collect_includes(interface_data)
    implementation_namespace = get_implementation_namespace(interface_data)
    export_namespace = get_namespace(interface_data)
    class_name = get_class_name(node_name)

    # Determine if QoS helpers are needed
    needs_qos_helpers = pub_needs_helpers or sub_needs_helpers

    # Render template
    return template.render(
        node_name=node_name,
        package_name=package_name,
        class_name=class_name,
        implementation_namespace=implementation_namespace,
        export_namespace=export_namespace,
        message_includes=message_includes,
        publishers=publishers,
        subscribers=subscribers,
        services=services,
        service_clients=service_clients,
        actions=actions,
        action_clients=action_clients,
        needs_qos_helpers=needs_qos_helpers,
    )


def generate_registration_cpp(interface_data: Dict[str, Any]) -> str:
    """Generate the component registration C++ file using Jinja2 template."""
    # Set up Jinja2 environment
    template_dir = Path(__file__).parent / "templates"
    env = Environment(loader=FileSystemLoader(template_dir), trim_blocks=False, lstrip_blocks=False)
    template = env.get_template("node_registration.cpp.jinja2")

    # Extract data
    node_name = interface_data["node"]["name"]
    package_name = interface_data["node"].get("package", "")
    export_namespace = get_namespace(interface_data)
    class_name = get_class_name(node_name)

    # Render template
    return template.render(
        node_name=node_name,
        package_name=package_name,
        class_name=class_name,
        export_namespace=export_namespace,
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

    # Prepare template data using generic entity preparation
    publishers, pub_qos_imports, pub_needs_helpers = prepare_python_entities(
        interface_data.get("publishers", []), ENTITY_CONFIGS[EntityKind.PUBLISHER]
    )
    subscribers, sub_qos_imports, sub_needs_helpers = prepare_python_entities(
        interface_data.get("subscribers", []), ENTITY_CONFIGS[EntityKind.SUBSCRIBER]
    )
    services, srv_qos_imports, _ = prepare_python_entities(
        interface_data.get("services", []), ENTITY_CONFIGS[EntityKind.SERVICE]
    )
    service_clients, cli_qos_imports, _ = prepare_python_entities(
        interface_data.get("service_clients", []), ENTITY_CONFIGS[EntityKind.SERVICE_CLIENT]
    )
    actions, _, _ = prepare_python_entities(interface_data.get("actions", []), ENTITY_CONFIGS[EntityKind.ACTION])
    action_clients, _, _ = prepare_python_entities(
        interface_data.get("action_clients", []), ENTITY_CONFIGS[EntityKind.ACTION_CLIENT]
    )

    # Collect all QoS imports
    qos_imports = pub_qos_imports | sub_qos_imports | srv_qos_imports | cli_qos_imports

    # Determine if QoS helpers are needed
    needs_qos_helpers = pub_needs_helpers or sub_needs_helpers

    # Collect unique import statements from all entity types
    message_imports: Set[str] = set()
    all_entities = [publishers, subscribers, services, service_clients, actions, action_clients]
    for entity_list in all_entities:
        for entity in entity_list:
            if entity.get("import_stmt"):
                message_imports.add(entity["import_stmt"])

    # Convert node_name to context class name
    class_name = get_class_name(node_name)
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
        services=services,
        service_clients=service_clients,
        actions=actions,
        action_clients=action_clients,
        needs_qos_helpers=needs_qos_helpers,
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
        parameters = get_dummy_parameter()

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
        parameters = get_dummy_parameter()

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


def generate_parameters_wrapper() -> str:
    """
    Generate parameters.py wrapper that flattens the _parameters module structure.
    """
    return """# auto-generated DO NOT EDIT

from ._parameters import parameters

# Flatten the nested structure for cleaner API
Params = parameters.Params
ParamListener = parameters.ParamListener

__all__ = ["Params", "ParamListener"]
"""


def generate_init_module() -> str:
    """
    Generate __init__.py that exposes interface and parameters as submodules.
    """
    return """# auto-generated DO NOT EDIT

from . import interface
from . import parameters

__all__ = ["interface", "parameters"]
"""


def main():
    parser = argparse.ArgumentParser(description="Generate cake node interface from YAML")
    parser.add_argument("interface_yaml", help="Path to interface.yaml file")
    parser.add_argument("--language", help="Target language (cpp or python)", choices=["cpp", "python"], required=True)
    parser.add_argument("--package", help="Package name to substitute for ${THIS_PACKAGE}", required=True)
    parser.add_argument("--node-name", help="Node name to substitute for ${THIS_NODE}", required=True)
    parser.add_argument("--output", help="Output directory path (for both C++ and Python)", required=True)

    args = parser.parse_args()

    # Read and parse YAML
    interface_path = Path(args.interface_yaml)
    if not interface_path.exists():
        print(f"Error: Interface file not found: {interface_path}", file=sys.stderr)
        sys.exit(1)

    with open(interface_path, "r") as f:
        interface_data = yaml.safe_load(f)

    # Validate YAML structure before processing
    try:
        validate_interface_yaml(interface_data)
    except InterfaceValidationError as e:
        print(f"Error: Validation failed for {interface_path}:", file=sys.stderr)
        print(f"  {e}", file=sys.stderr)
        sys.exit(1)

    # Substitute all template variables (${THIS_NODE}, ${THIS_PACKAGE}) in-place
    substitute_template_variables(interface_data, args.package, args.node_name)

    node_name = interface_data["node"]["name"]
    package_name = interface_data["node"].get("package", "")

    if args.language == "cpp":
        # Generate C++ header
        header_content = generate_header(interface_data)

        # Ensure output directory exists
        output_dir = Path(args.output)
        output_dir.mkdir(parents=True, exist_ok=True)

        # Generate files with node-name-based filenames
        header_file = output_dir / f"{node_name}_interface.hpp"
        with open(header_file, "w") as f:
            f.write(header_content)

        print(f"Generated: {header_file}")

        # Generate component registration .cpp file
        registration_content = generate_registration_cpp(interface_data)
        registration_file = output_dir / f"{node_name}_registration.cpp"
        with open(registration_file, "w") as f:
            f.write(registration_content)
        print(f"Generated: {registration_file}")

        # Always generate parameters.yaml (even if empty)
        if package_name:
            params_content = generate_parameters_yaml(interface_data, package_name, node_name)
            params_file = output_dir / f"{node_name}_interface.params.yaml"
            with open(params_file, "w") as f:
                f.write(params_content)
            print(f"Generated: {params_file}")

        # Generate processed interface YAML with plugin field
        interface_yaml_content = generate_interface_yaml(
            interface_path, interface_data, args.package, args.node_name, is_cpp=True
        )
        interface_yaml_file = output_dir / f"{node_name}.yaml"
        with open(interface_yaml_file, "w") as f:
            f.write(interface_yaml_content)
        print(f"Generated: {interface_yaml_file}")

    elif args.language == "python":
        # Generate Python module files
        output_dir = Path(args.output)
        output_dir.mkdir(parents=True, exist_ok=True)

        # Generate interface.py (main interface module)
        interface_content = generate_python_interface(interface_data)
        interface_file = output_dir / "interface.py"
        with open(interface_file, "w") as f:
            f.write(interface_content)
        print(f"Generated: {interface_file}")

        # Generate _parameters.py (from external library)
        parameters_content = generate_parameters_module(interface_data)
        parameters_internal_file = output_dir / "_parameters.py"
        with open(parameters_internal_file, "w") as f:
            f.write(parameters_content)
        print(f"Generated: {parameters_internal_file}")

        # Generate parameters.py (wrapper for clean API)
        parameters_wrapper_content = generate_parameters_wrapper()
        parameters_file = output_dir / "parameters.py"
        with open(parameters_file, "w") as f:
            f.write(parameters_wrapper_content)
        print(f"Generated: {parameters_file}")

        # Generate __init__.py (exposes submodules)
        init_content = generate_init_module()
        init_file = output_dir / "__init__.py"
        with open(init_file, "w") as f:
            f.write(init_content)
        print(f"Generated: {init_file}")

        # Generate processed interface YAML (without plugin field for Python)
        interface_yaml_content = generate_interface_yaml(
            interface_path, interface_data, args.package, args.node_name, is_cpp=False
        )
        interface_yaml_file = output_dir / f"{node_name}.yaml"
        with open(interface_yaml_file, "w") as f:
            f.write(interface_yaml_content)
        print(f"Generated: {interface_yaml_file}")


if __name__ == "__main__":
    main()
