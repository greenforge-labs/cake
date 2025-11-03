# cake

A collection of tools to make ROS2 development easier.

## Overview

Cake provides code generation tools that reduce boilerplate and make ROS2 node development more maintainable:

- **Node Interface Generator**: Automatically generate C++ node interfaces from YAML definitions
- **Type-safe Context**: Generated context structures with compile-time type checking
- **Declarative ROS2 Interfaces**: Define publishers, subscribers, services, service clients, and actions in YAML instead of C++
- **Flexible QoS Configuration**: Support for predefined profiles and custom parameters
- **Polling-based Actions**: Simple action server implementation designed for main-loop processing

## Quick Start

### 1. Set Up Your Package Structure

Organize your nodes in a `nodes/` directory:

```
my_package/
├── CMakeLists.txt
├── package.xml
└── nodes/
    └── my_node/
        ├── my_node.hpp
        ├── my_node.cpp
        ├── interface.yaml
        └── parameters.yaml (optional)
```

**Important:** Make sure your `package.xml` includes all required dependencies:
```xml
<buildtool_depend>ament_cmake_auto</buildtool_depend>

<depend>rclcpp</depend>
<depend>rclcpp_components</depend>
<depend>cake</depend>

<!-- Add your message/service dependencies -->
<depend>std_msgs</depend>
```

Dependencies listed in `package.xml` are automatically found and linked by `cake_auto_package()`. Note: You don't need to add `generate_parameter_library` - it's automatically found by cake.

### 2. Define Your Node Interface

Create `nodes/my_node/interface.yaml`:

```yaml
node:
    name: my_node
    package: ${THIS_PACKAGE}

parameters:
    update_rate:
        type: double
        default_value: 10.0
        description: "Update rate in Hz"

publishers:
    - topic: /cmd_vel
      type: geometry_msgs/msg/Twist
      qos: 10

subscribers:
    - topic: /odom
      type: nav_msgs/msg/Odometry
      qos: SensorDataQoS

services:
    - name: /reset
      type: std_srvs/srv/Trigger
```

### 3. Configure CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.22)
project(my_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(cake REQUIRED)

# Automatically handle all package setup
cake_auto_package()
```

### 4. Use the Generated Interface

```cpp
#include <my_package/my_node_interface.hpp>

namespace my_package::my_node {

struct MyContext : MyNodeContext<MyContext> {
    // Add your custom context members here
    // Parameters are automatically included in MyNodeContext
};

void init(std::shared_ptr<MyContext> ctx) {
    // Parameters are automatically loaded and ready to use
    RCLCPP_INFO(ctx->node->get_logger(), "Update rate: %.2f Hz", ctx->params.update_rate);

    // Publishers and subscribers are ready to use
    // ctx->publishers.cmd_vel
    // ctx->subscribers.odom

    // Set up service handler
    ctx->services.reset->set_request_handler(
        [](auto ctx, auto request, auto response) {
            response->success = true;
        }
    );
}

using MyNode = MyNodeBase<MyContext, init>;

} // namespace my_package::my_node

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MyNode>(rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}
```

## Interface YAML Reference

### Node Configuration

```yaml
node:
    name: my_node              # Node name (required) or ${THIS_NODE} for auto-detection
    package: ${THIS_PACKAGE}   # Package name or ${THIS_PACKAGE} for auto-detection
```

**Template Variables:**

- `${THIS_PACKAGE}` - Automatically replaced with the package name from CMake's `PROJECT_NAME`
- `${THIS_NODE}` - Automatically replaced with the node name (useful when using `cake_auto_package()` which auto-discovers nodes)

These placeholders are particularly useful when using `cake_auto_package()`, which automatically discovers and registers nodes from the `nodes/` directory.

### Parameters (Optional)

You can define ROS2 parameters directly in the `interface.yaml` file. These will be automatically processed by `generate_parameter_library`:

```yaml
parameters:
  update_rate:
    type: double
    default_value: 10.0
    description: "Update rate in Hz"
    validation:
      gt<>: [0.0]  # Must be greater than 0
  robot_name:
    type: string
    default_value: "robot1"
    description: "Name of the robot"
  joint_names:
    type: string_array
    default_value: ["joint1", "joint2", "joint3"]
    description: "List of joint names"
  read_only_param:
    type: bool
    default_value: true
    description: "A read-only parameter"
    read_only: true
```

**Supported Parameter Types:**
- `bool`, `int`, `double`, `string`
- Array types: `bool_array`, `int_array`, `double_array`, `string_array`
- Fixed-size types: `string_fixed_N`, `double_array_fixed_N` (where N is the size)

**Optional Fields:**
- `validation` - Constraints for the parameter value (e.g., `gt<>`, `lt<>`, `one_of<>`, etc.)
- `read_only` - If true, the parameter cannot be changed after initialization
- `additional_constraints` - Custom validation constraints

**Automatic Integration:**

Parameters are automatically integrated into your node's context:
- `ctx->params` - Contains all parameter values, automatically loaded before `init()` is called
- `ctx->param_listener` - The parameter listener instance (useful for runtime parameter updates)

You don't need to manually initialize parameters - they're ready to use immediately in your `init()` function.

**Note:** A parameters library is always generated for each node, even if no parameters are defined in `interface.yaml`. If no parameters are specified, a dummy parameter is automatically created to satisfy `generate_parameter_library` requirements.

For more details on parameter validation and advanced features, see the [generate_parameter_library documentation](https://github.com/PickNikRobotics/generate_parameter_library).

### Publishers

```yaml
publishers:
    - topic: /my_topic              # Topic name (required)
      type: std_msgs/msg/String     # Message type (required)
      qos: 10                       # QoS configuration (optional, default: 10)
      manually_created: false       # Skip auto-generation (optional, default: false)
```

### Subscribers

```yaml
subscribers:
    - topic: /other_topic           # Topic name (required)
      type: std_msgs/msg/String     # Message type (required)
      qos: 10                       # QoS configuration (optional, default: 10)
      manually_created: false       # Skip auto-generation (optional, default: false)
```

### Services

```yaml
services:
    - name: /my_service             # Service name (required)
      type: std_srvs/srv/Trigger    # Service type (required)
      qos: ServicesQoS              # QoS configuration (optional, omit to use C++ default)
      manually_created: false       # Skip auto-generation (optional, default: false)
```

When `qos` is omitted, the generated code will use the default QoS from rclcpp (`ServicesQoS`).

Services create service providers (servers) that respond to requests. To set the request handler:

```cpp
void init(std::shared_ptr<MyContext> ctx) {
    ctx->services.my_service->set_request_handler(
        [](auto ctx, auto request, auto response) {
            // Handle the service request
            response->success = true;
        }
    );
}
```

### Service Clients

```yaml
service_clients:
    - name: /my_service             # Service name (required)
      type: std_srvs/srv/Trigger    # Service type (required)
      qos: ServicesQoS              # QoS configuration (optional, omit to use C++ default)
      manually_created: false       # Skip auto-generation (optional, default: false)
```

When `qos` is omitted, the generated code will use the default QoS from rclcpp (`ServicesQoS`).

Service clients are exposed as raw `rclcpp::Client<ServiceT>::SharedPtr` for maximum flexibility:

```cpp
void init(std::shared_ptr<MyContext> ctx) {
    // Create request
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

    // Async call with callback
    auto future = ctx->service_clients.my_service->async_send_request(request);

    // Or with callback
    ctx->service_clients.my_service->async_send_request(
        request,
        [](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
            auto response = future.get();
            // Handle response
        }
    );
}
```

### Actions

```yaml
actions:
    - name: fibonacci                           # Action name (required)
      type: example_interfaces/action/Fibonacci # Action type (required)
      manually_created: false                   # Skip auto-generation (optional, default: false)
```

Actions use the `SingleGoalActionServer` which manages goal lifecycle and ensures only one goal is active at a time. The design is polling-based - you check for active goals in your main loop rather than using callbacks.

#### Setting Options

Configure action server behavior in your `init()` function:

```cpp
void init(std::shared_ptr<Context> ctx) {
    // Configure action server options
    cake::SingleGoalActionServerOptions<example_interfaces::action::Fibonacci> options;
    options.new_goals_replace_current_goal = false;  // Reject new goals if one is active
    options.goal_validator = [](const auto& goal) {
        return goal.order > 0;  // Custom validation logic
    };

    ctx->actions.fibonacci->set_options(options);
}
```

**Note:** Actions will reject all goals until options are set via `set_options()`.

#### Processing Goals (Polling Pattern)

Check for active goals in your node's main loop or timer callbacks:

```cpp
cake::create_timer(ctx, 100ms, [](auto ctx) {
    auto goal = ctx->actions.fibonacci->get_active_goal();
    if (!goal) {
        return;  // No active goal (or it was cancelled)
    }

    // Process goal incrementally
    auto feedback = std::make_shared<example_interfaces::action::Fibonacci::Feedback>();
    feedback->sequence.push_back(current_value);
    ctx->actions.fibonacci->publish_feedback(feedback);

    // Complete when done
    if (is_complete) {
        auto result = std::make_shared<example_interfaces::action::Fibonacci::Result>();
        result->sequence = final_sequence;
        ctx->actions.fibonacci->succeed(result);
    }
});
```

If the client cancels the goal, `get_active_goal()` will return nullptr on the next poll - no explicit cancellation handling needed.

#### Lifecycle Methods

- `get_active_goal()` - Returns current goal or nullptr if no active goal
- `publish_feedback(feedback)` - Send progress updates to client
- `succeed(result)` - Mark goal as succeeded and clear active goal
- `abort(result)` - Mark goal as aborted and clear active goal

## QoS Configuration

Cake supports three ways to configure QoS:

### 1. Simple Integer (Queue Depth)

Backward compatible with basic queue depth:

```yaml
publishers:
    - topic: /status
      type: std_msgs/msg/String
      qos: 10  # Queue depth
```

Generates: `create_publisher<T>("topic", 10)`

### 2. Predefined Profiles

Use ROS2's predefined QoS profiles:

```yaml
subscribers:
    - topic: /laser_scan
      type: sensor_msgs/msg/LaserScan
      qos: SensorDataQoS
```

Generates: `create_subscriber<T>("topic", rclcpp::SensorDataQoS())`

**Available Profiles:**
- `SensorDataQoS` - Best effort, volatile, depth 5 (for sensor data)
- `SystemDefaultsQoS` - System default settings
- `ParametersQoS` - Reliable, volatile, depth 1000 (for parameters)
- `ServicesQoS` - Reliable, volatile, depth 10 (for services)
- `ClockQoS` - Best effort, volatile, depth 1 (for clock)
- `RosoutQoS` - Reliable, transient local, depth 1000 (for logging)

### 3. Custom Parameters

Define custom QoS settings:

```yaml
publishers:
    - topic: /critical_data
      type: std_msgs/msg/String
      qos:
        reliability: reliable
        durability: transient_local
        depth: 100
```

Generates: `create_publisher<T>("topic", rclcpp::QoS(100).reliable().transient_local())`

**Supported Parameters:**

| Parameter | Options | Description |
|-----------|---------|-------------|
| `reliability` | `reliable`, `best_effort` | Delivery guarantee |
| `durability` | `volatile`, `transient_local` | Message persistence |
| `history` | `keep_last`, `keep_all` | History policy |
| `depth` | Integer | Queue size (for `keep_last`) |
| `deadline` | `{sec: X, nsec: Y}` | Max time between messages |
| `lifespan` | `{sec: X, nsec: Y}` | Max age of messages |
| `liveliness` | `automatic`, `manual_by_topic` | Liveness policy |

### 4. Profile with Overrides

Start with a profile and override specific parameters:

```yaml
subscribers:
    - topic: /reliable_sensor
      type: sensor_msgs/msg/LaserScan
      qos:
        profile: SensorDataQoS
        reliability: reliable  # Override default best_effort
        depth: 20             # Override default depth 5
```

Generates: `create_subscriber<T>("topic", rclcpp::SensorDataQoS().reliable().keep_last(20))`

## Advanced Features

### Manually Created Publishers/Subscribers

Skip auto-generation for specific topics when you need custom initialization:

```yaml
publishers:
    - topic: /custom_topic
      type: std_msgs/msg/String
      manually_created: true  # Completely excluded from code generation
```

Items marked with `manually_created: true` are completely excluded from the generated code, allowing you to create and initialize them entirely manually in your `init()` function or elsewhere in your code.

### Topic Name to Field Name Conversion

Topic names are automatically converted to valid C++ identifiers:

- `/cmd_vel` → `cmd_vel`
- `/robot/status` → `robot_status`
- Leading slashes are stripped
- Internal slashes become underscores

## Generated Code Structure

The generator creates:

### Publishers Struct
```cpp
template <typename ContextType> struct MyNodePublishers {
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr my_topic;
};
```

### Subscribers Struct
```cpp
template <typename ContextType> struct MyNodeSubscribers {
    std::shared_ptr<cake::Subscriber<std_msgs::msg::String, ContextType>> my_topic;
};
```

### Services Struct
```cpp
template <typename ContextType> struct MyNodeServices {
    std::shared_ptr<cake::Service<std_srvs::srv::Trigger, ContextType>> my_service;
};
```

### Service Clients Struct
```cpp
template <typename ContextType> struct MyNodeServiceClients {
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr my_service;
};
```

### Context Struct
```cpp
template <typename DerivedContextType> struct MyNodeContext : cake::Context {
    MyNodePublishers<DerivedContextType> publishers;
    MyNodeSubscribers<DerivedContextType> subscribers;
    MyNodeServices<DerivedContextType> services;
    MyNodeServiceClients<DerivedContextType> service_clients;
    std::shared_ptr<ParamListener> param_listener;  // Automatically initialized
    Params params;                                   // Automatically loaded
};
```

### Base Node Class
```cpp
template <typename ContextType, auto init_func, auto extend_options = ...>
class MyNodeBase : public cake::BaseNode<"my_node", extend_options> {
    // Initialization code that creates publishers/subscribers
};
```

## Examples

See the `cake_example` package for a complete working example.

## Testing

The code generator includes comprehensive tests. See [tests/README.md](cake/tests/README.md) for details on running and adding tests.

## CMake API

### `cake_auto_package()`

Completely automates cake package setup - handles dependencies, library creation, node registration, and package finalization.

**What it does:**
- Finds all build dependencies via `ament_auto_find_build_dependencies()`
- Creates the main SHARED library from all source files in `nodes/`
- Sets C++20 as the required C++ standard
- Scans the `nodes/` directory for node subdirectories
- For each node found:
  - Generates interface library if `interface.yaml` exists
  - Generates parameters library if `parameters.yaml` exists
  - Links both libraries to `${PROJECT_NAME}`
  - Registers the node as an rclcpp component with the correct naming convention
- Finalizes the package with `ament_auto_package()`

**Requirements:**
- Must call `find_package(cake REQUIRED)` first
- Nodes must be organized in a `nodes/` directory at the project root
- Each node has its own subdirectory (e.g., `nodes/my_node/`)
- Node directory names must be in snake_case (e.g., `my_node`)

**Dependencies:**

`cake_auto_package()` uses `ament_cmake_auto` to automatically find and link dependencies, but **you must declare them in your `package.xml`** for this to work.

Example `package.xml`:
```xml
<?xml version="1.0"?>
<package format="3">
  <name>my_package</name>
  <version>0.0.0</version>
  <description>My cake-based package</description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake_auto</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>rclcpp_components</depend>
  <depend>cake</depend>

  <!-- Add your message/service dependencies here -->
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

The `ament_auto_find_build_dependencies()` call inside `cake_auto_package()` will automatically find and link all packages listed as `<depend>`, `<build_depend>`, `<build_export_depend>`, or `<exec_depend>` in your `package.xml`. You don't need to manually call `find_package()` or `target_link_libraries()` for these dependencies.

**Note:** You don't need to add `generate_parameter_library` to your `package.xml` - it's automatically found by `cake_auto_package()` since it's an internal implementation detail of cake.

**Conventions Enforced:**
- **Main library**: Named `${PROJECT_NAME}`, type SHARED
- **C++ standard**: C++20
- **Source location**: `nodes/` directory
- **Node directory**: snake_case (e.g., `my_node`)
- **C++ namespace**: `${PROJECT_NAME}::my_node`
- **C++ class name**: PascalCase (e.g., `MyNode`, auto-converted)
- **Plugin class**: `${PROJECT_NAME}::my_node::MyNode`
- **Interface library**: `my_node_interface`
- **Parameters library**: `my_node_parameters`
- **Executable**: `my_node`

**Example CMakeLists.txt:**
```cmake
cmake_minimum_required(VERSION 3.22)
project(my_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(cake REQUIRED)

# Automatically handle all package setup
cake_auto_package()
```

**Before (Manual):**
```cmake
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED DIRECTORY nodes)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)

# For each node, you had to write:
cake_generate_node_interface(my_node_interface nodes/my_node/interface.yaml)
target_link_libraries(${PROJECT_NAME} my_node_interface)

generate_parameter_library(my_node_parameters nodes/my_node/parameters.yaml)
target_link_libraries(${PROJECT_NAME} my_node_parameters)

rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN "my_package::my_node::MyNode"
    EXECUTABLE "my_node")

ament_auto_package()
```

**After (Automatic):**
```cmake
find_package(cake REQUIRED)

cake_auto_package()  # Handles everything!
```

**File Structure Example:**
```
my_package/
├── CMakeLists.txt
├── package.xml
└── nodes/
    ├── my_node/
    │   ├── my_node.hpp
    │   ├── my_node.cpp
    │   ├── interface.yaml
    │   └── parameters.yaml
    └── another_node/
        ├── another_node.hpp
        ├── another_node.cpp
        ├── interface.yaml
        └── parameters.yaml
```

With this structure, `cake_auto_package()` will automatically register both `my_node` and `another_node` with all their interfaces and parameters.

---

### `cake_generate_node_interface(LIB_NAME YAML_FILE)`

Generates a C++ header from a YAML interface definition.

**Parameters:**
- `LIB_NAME`: Name of the interface library target
- `YAML_FILE`: Path to interface.yaml file (relative to current source dir)

**Example:**
```cmake
cake_generate_node_interface(my_node_interface nodes/my_node/interface.yaml)
```

This creates an interface library target and installs the generated header to `include/${PROJECT_NAME}/${LIB_NAME}.hpp`.

**Note:** If you're using `cake_auto_package()`, you don't need to call this manually - it's handled automatically.
