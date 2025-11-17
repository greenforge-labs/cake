# cake

A collection of tools to make ROS2 development easier.

## Overview

Cake provides code generation tools that reduce boilerplate and make ROS2 node development more maintainable:

- **Automatic Build System**: Single `cake_auto_package()` call handles dependencies, code generation, and component registration
- **No Boilerplate**: No need to write main() functions or component registration macros for C++ nodes
- **Node Interface Generator**: Automatically generate C++ and Python node interfaces from YAML definitions
- **Multi-language Support**: Write nodes in C++, Python, or mix both in the same package
- **Type-safe Context**: Generated context structures with compile-time (C++) and runtime (Python) type safety
- **Declarative ROS2 Interfaces**: Define publishers, subscribers, services, service clients, and actions in YAML
- **Flexible QoS Configuration**: Support for predefined profiles and custom parameters
- **Polling-based Actions**: Simple action server implementation designed for main-loop processing
- **Automatic Parameter Generation**: Parameters defined in YAML are automatically integrated into your node

## Quick Start

### 1. Set Up Your Package Structure

Organize your nodes in a `nodes/` directory:

**C++ Node:**
```
my_package/
├── CMakeLists.txt
├── package.xml
└── nodes/
    └── my_node/
        ├── my_node.hpp
        ├── my_node.cpp
        └── interface.yaml
```

**Python Node:**
```
my_package/
├── CMakeLists.txt
├── package.xml
└── nodes/
    └── my_python_node/
        ├── my_python_node.py
        └── interface.yaml
```

**Note:** Parameters are defined directly in `interface.yaml` - there's no separate parameters file needed.

**Important:** Make sure your `package.xml` includes all required dependencies:

**For C++ nodes:**
```xml
<depend>cake</depend>
<depend>rclcpp</depend>

<!-- Add your message/service dependencies -->
<depend>std_msgs</depend>
```

**For Python nodes:**
```xml
<depend>cake</depend>
<depend>rclpy</depend>

<!-- Add your message/service dependencies -->
<depend>std_msgs</depend>
```

**For packages with both C++ and Python nodes:**
```xml
<depend>cake</depend>
<depend>rclcpp</depend>
<depend>rclpy</depend>

<!-- Add your message/service dependencies -->
<depend>std_msgs</depend>
```

Dependencies listed in `package.xml` are automatically found and linked by `cake_auto_package()`.

**Note:** You don't need to add `generate_parameter_library` - it's automatically found by cake.

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

**Note:** If you have `launch/` or `config/` directories in your package root, `cake_auto_package()` will automatically install them to `share/${PROJECT_NAME}/`. See the [CMake API](#cmake-api) section for details on installing additional directories.

### 4. Implement Your Node

**C++ Node (`nodes/my_node/my_node.hpp`):**

```cpp
#include <my_package/my_node_interface.hpp>

namespace my_package::my_node {

struct Context : MyNodeContext<Context> {
    // Add your custom context members here
    int my_counter = 0;
};

void init(std::shared_ptr<Context> ctx);

// Define the node type
using MyNode = MyNodeBase<Context, init>;

} // namespace my_package::my_node
```

**C++ Implementation (`nodes/my_node/my_node.cpp`):**

```cpp
#include "my_node.hpp"

namespace my_package::my_node {

void init(std::shared_ptr<Context> ctx) {
    // Parameters are automatically loaded and ready to use
    RCLCPP_INFO(ctx->node->get_logger(), "Update rate: %.2f Hz", ctx->params.update_rate);

    // Publishers and subscribers are ready to use
    // ctx->publishers.cmd_vel
    // ctx->subscribers.odom

    // Set up service handler
    ctx->services.reset->set_request_handler(
        [](auto ctx, auto request, auto response) {
            response->success = true;
            ctx->my_counter++;
        }
    );
}

} // namespace my_package::my_node
```

**That's it!** No main() function needed, no component registration macro needed. The node is automatically:
- Compiled into a shared library
- Registered as an rclcpp component
- Given an executable wrapper for `ros2 run`

**Run your node:**
```bash
ros2 run my_package my_node
```

**Or load as a component:**
```bash
ros2 component load /ComponentManager my_package my_package::MyNode
```

### 5. Python Usage

For Python nodes, cake generates a clean interface structure:

```python
from my_package.my_python_node.interface import PythonNodeContext, run
from my_package.my_python_node.parameters import Params, ParamListener
import cake

class Context(PythonNodeContext):
    # Add your custom context members here
    my_value: float = 0.0

def init(ctx: Context):
    # Parameters are automatically loaded
    ctx.logger.info(f"Update rate: {ctx.params.update_rate}")

    # Set up subscriber callback
    ctx.subscribers.odom.set_callback(handle_odom)

    # Create a thread for background work
    cake.create_thread(ctx, background_thread)

def handle_odom(ctx: Context, msg):
    ctx.logger.info(f"Received odometry: {msg.pose}")
    # Publish using the generated publisher
    ctx.publishers.cmd_vel.publish(twist_msg)

def background_thread(ctx: Context):
    while rclpy.ok():
        ctx.my_value += 1
        time.sleep(0.1)

if __name__ == "__main__":
    run(Context, init)
```

**Using Services (Python):**
```python
def init(ctx: Context):
    # Set up service request handler
    ctx.services.reset.set_request_handler(handle_reset)

def handle_reset(ctx: Context, request, response):
    # Handle the service request
    ctx.logger.info("Reset service called")
    response.success = True
    response.message = "System reset successful"
    # Access custom context members
    ctx.my_value = 0.0
```

**Using Service Clients (Python):**
```python
def init(ctx: Context):
    # Service clients are ready to use immediately
    # Call service asynchronously
    request = Trigger.Request()
    future = ctx.service_clients.trigger_service.call_async(request)
    future.add_done_callback(lambda f: handle_service_response(ctx, f))

def handle_service_response(ctx: Context, future):
    try:
        response = future.result()
        ctx.logger.info(f"Service response: {response.message}")
    except Exception as e:
        ctx.logger.error(f"Service call failed: {e}")
```

**Generated Python API:**
- `interface.py` - Contains context class and `run()` function
- `parameters.py` - Exposes `Params` and `ParamListener`
- `__init__.py` - Makes the module importable

**Import patterns:**
```python
# Direct imports (recommended)
from my_package.my_node.interface import PythonNodeContext, run
from my_package.my_node.parameters import Params, ParamListener

# Or via submodule
from my_package.my_node import interface, parameters
interface.run(Context, init)
```

## C++ Namespace Structure

Cake uses a two-level namespace structure that separates implementation from component registration:

### Implementation Namespace

Your node implementation lives in a nested namespace to avoid collisions between different nodes:

```cpp
namespace my_package::my_node {
    struct Context : MyNodeContext<Context> { /* ... */ };
    void init(std::shared_ptr<Context> ctx);
    using MyNode = MyNodeBase<Context, init>;
}
```

**Format:** `package_name::node_name`

This keeps all implementation details scoped and prevents naming conflicts when multiple nodes define similar types (e.g., multiple nodes with a `Context` struct).

### Component Registration Namespace

Components are registered and exported with a simplified namespace:

```cpp
// Automatically generated in *_registration.cpp
namespace my_package {
    using MyNode = my_package::my_node::MyNode;
}
RCLCPP_COMPONENTS_REGISTER_NODE(my_package::MyNode);
```

**Format:** `package_name::NodeName`

### How It Works

Cake uses C++ type aliases to expose your nested implementation class at the package level for component registration. This gives you:

✅ **Clean component names**: `my_package::MyNode` instead of `my_package::my_node::MyNode`
✅ **No name collisions**: Implementation stays safely scoped in `my_package::my_node`
✅ **Zero user changes**: Your code continues to use the nested namespace as before

### Discovering Components

List available components in a package:

```bash
$ ros2 component types | grep my_package
my_package
  my_package::MyNode
  my_package::AnotherNode
```

Load a component:

```bash
ros2 component load /ComponentManager my_package my_package::MyNode
```

Or use `ros2 component standalone`:

```bash
ros2 component standalone my_package my_package::MyNode
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

### Actions (Servers)

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

### Action Clients

```yaml
action_clients:
    - name: /navigate                           # Action name (required)
      type: nav2_msgs/action/NavigateToPose     # Action type (required)
      manually_created: false                   # Skip auto-generation (optional, default: false)
```

Action clients are exposed as raw `rclcpp_action::Client<ActionT>::SharedPtr` for maximum flexibility, giving you full access to the rclcpp_action API.

```cpp
void init(std::shared_ptr<MyContext> ctx) {
    // Create goal message
    auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.pose.position.x = 1.0;
    goal_msg.pose.pose.position.y = 2.0;

    // Send goal asynchronously
    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();

    // Optional: Set goal response callback
    send_goal_options.goal_response_callback =
        [](auto goal_handle) {
            if (!goal_handle) {
                RCLCPP_ERROR(rclcpp::get_logger("my_node"), "Goal was rejected");
            } else {
                RCLCPP_INFO(rclcpp::get_logger("my_node"), "Goal accepted");
            }
        };

    // Optional: Set feedback callback
    send_goal_options.feedback_callback =
        [](auto, const auto& feedback) {
            RCLCPP_INFO(rclcpp::get_logger("my_node"),
                       "Distance remaining: %.2f",
                       feedback->distance_remaining);
        };

    // Optional: Set result callback
    send_goal_options.result_callback =
        [](const auto& result) {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_INFO(rclcpp::get_logger("my_node"), "Navigation succeeded!");
            }
        };

    // Send the goal
    ctx->action_clients.navigate->async_send_goal(goal_msg, send_goal_options);
}
```

For full details on the action client API, see the [rclcpp_action documentation](https://docs.ros2.org/latest/api/rclcpp_action/).

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

### Actions Struct
```cpp
template <typename ContextType> struct MyNodeActions {
    std::shared_ptr<cake::SingleGoalActionServer<example_interfaces::action::Fibonacci>> fibonacci;
};
```

### Action Clients Struct
```cpp
template <typename ContextType> struct MyNodeActionClients {
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigate;
};
```

### Context Struct
```cpp
template <typename DerivedContextType> struct MyNodeContext : cake::Context {
    MyNodePublishers<DerivedContextType> publishers;
    MyNodeSubscribers<DerivedContextType> subscribers;
    MyNodeServices<DerivedContextType> services;
    MyNodeServiceClients<DerivedContextType> service_clients;
    MyNodeActions<DerivedContextType> actions;
    MyNodeActionClients<DerivedContextType> action_clients;
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

Completely automates cake package setup - handles dependencies, library creation, node registration, and package finalization for both C++ and Python nodes.

**Parameters:**
- `INSTALL_TO_SHARE` - (Optional) List of directories to install to `share/${PROJECT_NAME}/`

**Automatic Directory Detection:**

`cake_auto_package()` automatically detects and installs common ROS2 directories if they exist in your package root:
- `launch/` - Launch files for ros2 launch
- `config/` - Configuration files (YAML, JSON, etc.)

These directories are automatically installed to `share/${PROJECT_NAME}/` without requiring any additional configuration. You can still use `INSTALL_TO_SHARE` to add additional directories beyond these automatically detected ones.

**Automatic Interface Installation:**

`cake_auto_package()` automatically processes and installs all `interface.yaml` files to `share/${PROJECT_NAME}/interfaces/` for documentation and static analysis purposes:

**Per-Node Interfaces:**
- Each node's `interface.yaml` is automatically renamed and installed as `${NODE_NAME}.yaml`
- Token replacement is performed:
  - `${THIS_NODE}` → actual node name
  - `${THIS_PACKAGE}` → actual package name
- Example: `nodes/my_node/interface.yaml` → `share/my_package/interfaces/my_node.yaml`

**Package-Level Interfaces (Optional):**
- You can create an `interfaces/` directory in your package root to document external nodes or nodes implemented without cake
- Files are installed as-is to `share/${PROJECT_NAME}/interfaces/` **without token replacement**
- Useful for documenting:
  - Nodes from other packages referenced in your launch files
  - Legacy nodes not using cake
  - Third-party node interfaces

**Conflict Detection:**
- If a package-level interface file has the same name as a per-node interface (e.g., both `interfaces/my_node.yaml` and `nodes/my_node/interface.yaml` exist), the build will fail with a clear error message
- This prevents accidental overwrites and ensures explicit naming

**Usage:**
```cmake
# Minimal usage - automatically installs launch/ and config/ if they exist
cake_auto_package()

# Install additional directories beyond auto-detected ones
cake_auto_package(
  INSTALL_TO_SHARE
    rviz
    meshes
    urdf
)
```

**What it does:**
- Finds all build dependencies via `ament_auto_find_build_dependencies()`
- Detects languages used in `nodes/` directory (C++ and/or Python)
- **For C++ nodes:**
  - Creates the main SHARED library from all `.cpp` files in `nodes/`
  - Sets C++20 as the required C++ standard
  - Automatically generates component registration code
- **For Python nodes:**
  - Sets up Python package structure
  - Creates top-level `__init__.py`
- Scans the `nodes/` directory for node subdirectories
- **For each node found:**
  - Validates that `interface.yaml` exists (REQUIRED)
  - Detects node language (C++ or Python)
  - Generates interface code from `interface.yaml`
  - Generates parameters library (always, even if no parameters defined)
  - **For C++ nodes:**
    - Generates interface header (`${NODE_NAME}_interface.hpp`)
    - Generates parameters library from interface.yaml
    - Generates component registration code (`${NODE_NAME}_registration.cpp`)
    - Creates interface library target
    - Creates parameters library target
    - Links all libraries to `${PROJECT_NAME}`
    - Registers as rclcpp_component with executable wrapper
  - **For Python nodes:**
    - Generates `interface.py`, `_parameters.py`, `parameters.py`, `__init__.py`
    - Installs user Python files
    - Creates executable wrapper using `runpy.run_module()`
- Automatically detects and installs `launch/` and `config/` directories if present
- Processes and installs all `interface.yaml` files to `share/${PROJECT_NAME}/interfaces/`:
  - Per-node interfaces: renamed to `${NODE_NAME}.yaml` with token replacement
  - Package-level interfaces (from `interfaces/` directory): installed as-is without token replacement
  - Detects and fails on naming conflicts between sources
- Installs any additional directories specified via `INSTALL_TO_SHARE` parameter
- Finalizes the package with `ament_auto_package()`

**Requirements:**
- Must call `find_package(cake REQUIRED)` first
- Nodes must be organized in a `nodes/` directory at the project root
- Each node has its own subdirectory (e.g., `nodes/my_node/`)
- Node directory names must be in snake_case (e.g., `my_node`)
- Each node directory must contain `interface.yaml`
- **C++ nodes:** Must contain at least one `.cpp` file
- **Python nodes:** Must contain at least one `.py` file (typically `<node_name>.py`)
- Mixed language nodes (both C++ and Python) are not supported

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

  <depend>cake</depend>
  <depend>rclcpp</depend>

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

**C++ Nodes:**
- **Main library**: Named `${PROJECT_NAME}`, type SHARED
- **C++ standard**: C++20
- **Source location**: `nodes/` directory
- **Node directory**: snake_case (e.g., `my_node`)
- **Implementation namespace**: `${PROJECT_NAME}::node_name` (e.g., `my_package::my_node`)
- **Export namespace**: `${PROJECT_NAME}` (e.g., `my_package`)
- **C++ class name**: PascalCase (e.g., `MyNode`, auto-converted from snake_case)
- **Plugin class**: `${PROJECT_NAME}::MyNode` (registered via type alias from implementation namespace)
- **Interface library**: `my_node_interface`
- **Parameters library**: `my_node_parameters`
- **Registration file**: `my_node_registration.cpp` (auto-generated, includes type alias)
- **Executable**: `my_node`
- **Component registration**: Automatic via generated registration file

**Python Nodes:**
- **Package structure**: `${PROJECT_NAME}.${node_name}`
- **Node directory**: snake_case (e.g., `my_node`)
- **Context class name**: PascalCase with `Context` suffix (e.g., `MyNodeContext`)
- **Generated files**: `interface.py`, `_parameters.py`, `parameters.py`, `__init__.py`
- **Executable**: `my_node` (wrapper script using `runpy.run_module()`)
- **Installation**: `lib/${PROJECT_NAME}/${node_name}` for executable, `lib/python3.X/site-packages/${PROJECT_NAME}/${node_name}/` for Python files

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

# For each node, you had to:
# 1. Generate the interface
cake_generate_node_interface(my_node_interface nodes/my_node/interface.yaml)
target_link_libraries(${PROJECT_NAME} my_node_interface)

# 2. Generate parameters
generate_parameter_library(my_node_parameters nodes/my_node/parameters.yaml)
target_link_libraries(${PROJECT_NAME} my_node_parameters)

# 3. Register the component (in your C++ file):
# #include <rclcpp_components/register_node_macro.hpp>
# namespace my_package {
#     using MyNode = my_package::my_node::MyNode;
# }
# RCLCPP_COMPONENTS_REGISTER_NODE(my_package::MyNode);

# 4. Create the executable
rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN "my_package::MyNode"
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
├── interfaces/                # Optional: document external/legacy nodes
│   ├── external_node.yaml
│   └── legacy_node.yaml
└── nodes/
    ├── cpp_node/              # C++ node
    │   ├── cpp_node.hpp
    │   ├── cpp_node.cpp
    │   └── interface.yaml
    ├── python_node/           # Python node
    │   ├── python_node.py
    │   └── interface.yaml
    └── another_cpp_node/      # Another C++ node
        ├── another_cpp_node.hpp
        ├── another_cpp_node.cpp
        └── interface.yaml
```

With this structure, `cake_auto_package()` will automatically:
- Detect and build C++ nodes as components in the shared library
- Set up Python nodes with their package structure and executables
- Generate interfaces and parameters for all nodes
- Register executables for all nodes (both C++ and Python)
- Install all interface files to `share/my_package/interfaces/`:
  - `cpp_node.yaml`, `python_node.yaml`, `another_cpp_node.yaml` (from nodes/, with token replacement)
  - `external_node.yaml`, `legacy_node.yaml` (from interfaces/, without token replacement)

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
