# cake

**Declarative code generation for ROS 2 nodes**

Cake transforms simple YAML interface definitions into strongly-typed C++ and Python ROS 2 lifecycle node scaffolding. Define your publishers, subscribers, services, actions and parameters in one simple file and Cake will handle the rest.

Using cake, writing ROS2 nodes becomes a... <sub>piece of cake (hehe)</sub>

## Quick Start

Cake uses a convention-over-configuration approach with automatic build system integration. Here's how to create a complete ROS 2 package in minutes:

### 1. Create Package Structure
Your package should follow this structure:
```
my_package/
├── nodes/
│   └── my_node/
│       ├── interface.yaml    # Interface definition
│       ├── my_node.hpp       # Header (C++ only)
│       └── my_node.cpp       # Implementation (.cpp for C++, .py for Python)
├── CMakeLists.txt
└── package.xml
```

### 2. Define Your Node Interface

Create `nodes/my_node/interface.yaml`:

```yaml
parameters:
    important_parameter:
        type: string
        default_value: "oh hi mark"
        description: "A very important string."

publishers:
    - topic: some_topic
      type: std_msgs/msg/String
      qos:
        history: 10
        reliability: RELIABLE

subscribers:
    - topic: other_topic
      type: std_msgs/msg/Bool
      qos:
        history: 5
        reliability: BEST_EFFORT

services:
    - name: my_service
      type: example_interfaces/srv/AddTwoInts
```

### 3. Implement Your Node

#### C++ Example

First, create the header (`nodes/my_node/my_node.hpp`):

> **Design Pattern:** Cake uses a lifecycle `Session` class rather than subclassing `rclcpp_lifecycle::LifecycleNode`. This separation makes testing easier (you can test logic without spinning up ROS), keeps state explicit, and allows callbacks to be simple free functions. The `Session` is created during the `on_configure` lifecycle transition and destroyed on `cleanup`/`shutdown`. To define the `Session` of your node, you subclass the auto-generated `<NodeName>Session` struct and add your own variables to it. The auto-generated `Session` class will contain a reference to the lifecycle node instance, as well as all publishers, subscribers, services, actions and parameters.

```cpp
#pragma once

#include <memory>
#include <my_package/my_node_interface.hpp>

namespace my_package::my_node {

// Extend the generated session with custom state
struct Session : MyNodeSession<Session> {
    using MyNodeSession::MyNodeSession;
    // Add any custom state here
    int my_counter = 0;
};

// Forward declare on_configure function
CallbackReturn on_configure(std::shared_ptr<Session> sn);

// Define the node class using the generated base
// This must match the pattern: package::node_name::NodeName
using MyNode = MyNodeBase<Session, on_configure>;

} // namespace my_package::my_node
```

Then implement it (`nodes/my_node/my_node.cpp`):

> **Design Pattern:** Cake uses a free function `on_configure()` approach instead of subclassing `rclcpp_lifecycle::LifecycleNode`. The `on_configure()` function receives a fully-constructed session with all publishers, subscribers, and parameters ready to use. This functional approach, coupled with the session object, makes nodes easier to reason about, simpler to write and more testable. By storing a reference to the lifecycle node in the session, we create a "has-a" relationship with the Node rather than "is-a", cleanly separating ROS communication from your implementation logic.

```cpp
#include "my_node.hpp"

namespace my_package::my_node {

void msg_callback(std::shared_ptr<Session> sn, std_msgs::msg::Bool::ConstSharedPtr msg) {
    sn->my_counter++;
    RCLCPP_INFO(sn->node.get_logger(), "Got a bool: %d (count: %d)", msg->data, sn->my_counter);
}

void addition_request_handler(
    std::shared_ptr<Session> sn,
    example_interfaces::srv::AddTwoInts::Request::SharedPtr request,
    example_interfaces::srv::AddTwoInts::Response::SharedPtr response
) {
    response->sum = request->a + request->b;
}

CallbackReturn on_configure(std::shared_ptr<Session> sn) {
    // Access parameters
    RCLCPP_INFO(sn->node.get_logger(), "important_parameter: %s", sn->params.important_parameter.c_str());

    // Publish messages
    auto msg = std_msgs::msg::String();
    msg.data = sn->params.important_parameter;
    sn->publishers.some_topic->publish(msg);

    // Set callbacks
    sn->subscribers.other_topic->set_callback(msg_callback);
    sn->services.my_service->set_request_handler(addition_request_handler);

    return CallbackReturn::SUCCESS;
}

} // namespace my_package::my_node
```

#### Python Example (`nodes/my_node/my_node.py`)

```python
from dataclasses import dataclass

from cake import TransitionCallbackReturn
from my_package.my_node import MyNodeSession, run
from std_msgs.msg import String

# Extend the generated session with custom state
@dataclass
class MySession(MyNodeSession):
    my_counter: int = 0

def msg_callback(sn: MySession, msg):
    sn.my_counter += 1
    sn.logger.info(f"Got a bool: {msg.data} (count: {sn.my_counter})")

def on_configure(sn: MySession) -> TransitionCallbackReturn:
    # Access parameters
    sn.logger.info(f"important_parameter: {sn.params.important_parameter}")

    # Publish messages
    msg = String()
    msg.data = sn.params.important_parameter
    sn.publishers.some_topic.publish(msg)

    # Set callbacks
    sn.subscribers.other_topic.set_callback(msg_callback)

    return TransitionCallbackReturn.SUCCESS

if __name__ == "__main__":
    run(MySession, on_configure)
```

### 4. Create CMakeLists.txt

This is all you need in your `CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.22)
project(my_package)

find_package(cake REQUIRED)
cake_auto_package()
```

**Note:** Cake assumes a single-threaded executor. See [Threading Model](#threading-model) for details.

That's it! `cake_auto_package()` automatically:
- Detects C++ and Python nodes in the `nodes/` folder
- Generates interfaces and parameter libraries
- Builds libraries and executables
- Registers components for C++ nodes
- Installs everything correctly

### 5. Create package.xml

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_package</name>
  <version>0.0.0</version>
  <description>My cake package</description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>Apache 2.0</license>

  <depend>cake</depend>
  <depend>rclcpp</depend>  <!-- For C++ nodes -->
  <depend>rclpy</depend>   <!-- For Python nodes -->

  <!-- Add your message dependencies -->
  <depend>std_msgs</depend>
  <depend>example_interfaces</depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### 6. Build and Run

```bash
cd ~/ros2_ws
colcon build --packages-select my_package
source install/setup.bash

# Run as executable
ros2 run my_package my_node

# Or load as component (if written in C++)
ros2 component standalone my_package my_package::MyNode
```

## Lifecycle Callbacks

Cake nodes follow the standard ROS 2 lifecycle state machine. The Quick Start examples show `on_configure`, but there are five lifecycle callbacks you can implement. Only `on_configure` is required — the rest are optional.

### Available Callbacks

| Callback | Return Type | Required | Transition |
|----------|-------------|----------|------------|
| `on_configure` | `CallbackReturn` | **Yes** | Unconfigured → Inactive |
| `on_activate` | `CallbackReturn` | No | Inactive → Active |
| `on_deactivate` | `CallbackReturn` | No | Active → Inactive |
| `on_cleanup` | `CallbackReturn` | No | Inactive → Unconfigured |
| `on_shutdown` | `void` | No | Any → Finalized |

All callbacks except `on_shutdown` can return `SUCCESS`, `FAILURE`, or `ERROR`. Returning `FAILURE` rejects the transition and the node stays in its previous state. Returning `ERROR` transitions the node to the `Finalized` (error) state.

### Execution Order

Cake handles entity lifecycle management automatically around your callbacks:

- **Configure:** Session is created with all entities, then `on_configure` runs. On failure, the session is destroyed.
- **Activate:** `on_activate` runs first, then entities are activated automatically.
- **Deactivate:** `on_deactivate` runs first, then entities are deactivated automatically.
- **Cleanup:** `on_cleanup` runs first. On success, the session is destroyed.
- **Shutdown:** `on_shutdown` runs, then the session is always destroyed regardless of outcome.

### C++ Example

Callbacks are passed as template parameters to the generated `Base` class. Optional callbacks default to returning `SUCCESS` (or no-op for `on_shutdown`):

```cpp
#pragma once
#include <my_package/my_node_interface.hpp>

namespace my_package::my_node {

struct Session : MyNodeSession<Session> {
    using MyNodeSession::MyNodeSession;
    bool is_running = false;
};

CallbackReturn on_configure(std::shared_ptr<Session> sn);
CallbackReturn on_activate(std::shared_ptr<Session> sn);
CallbackReturn on_deactivate(std::shared_ptr<Session> sn);
CallbackReturn on_cleanup(std::shared_ptr<Session> sn);
void on_shutdown(std::shared_ptr<Session> sn);

// Pass all callbacks as template parameters (only on_configure is required)
using MyNode = MyNodeBase<Session, on_configure, on_activate, on_deactivate, on_cleanup, on_shutdown>;

} // namespace my_package::my_node
```

```cpp
#include "my_node.hpp"

namespace my_package::my_node {

CallbackReturn on_configure(std::shared_ptr<Session> sn) {
    RCLCPP_INFO(sn->node.get_logger(), "Configuring...");
    sn->subscribers.sensor->set_callback(sensor_callback);
    return CallbackReturn::SUCCESS;
}

CallbackReturn on_activate(std::shared_ptr<Session> sn) {
    RCLCPP_INFO(sn->node.get_logger(), "Activating...");
    sn->is_running = true;
    return CallbackReturn::SUCCESS;
}

CallbackReturn on_deactivate(std::shared_ptr<Session> sn) {
    RCLCPP_INFO(sn->node.get_logger(), "Deactivating...");
    sn->is_running = false;
    return CallbackReturn::SUCCESS;
}

CallbackReturn on_cleanup(std::shared_ptr<Session> sn) {
    RCLCPP_INFO(sn->node.get_logger(), "Cleaning up...");
    return CallbackReturn::SUCCESS;
}

void on_shutdown(std::shared_ptr<Session> sn) {
    RCLCPP_INFO(sn->node.get_logger(), "Shutting down...");
}

} // namespace my_package::my_node
```

You can also provide only the callbacks you need — omitted ones use sensible defaults:

```cpp
// Only on_configure and on_activate
using MyNode = MyNodeBase<Session, on_configure, on_activate>;
```

### Python Example

Callbacks are passed as keyword arguments to `run()`. Optional callbacks are simply omitted:

```python
from cake import TransitionCallbackReturn
from my_package.my_node import MyNodeSession, run

@dataclass
class MySession(MyNodeSession):
    is_running: bool = False

def on_configure(sn: MySession) -> TransitionCallbackReturn:
    sn.logger.info("Configuring...")
    sn.subscribers.sensor.set_callback(sensor_callback)
    return TransitionCallbackReturn.SUCCESS

def on_activate(sn: MySession) -> TransitionCallbackReturn:
    sn.logger.info("Activating...")
    sn.is_running = True
    return TransitionCallbackReturn.SUCCESS

def on_deactivate(sn: MySession) -> TransitionCallbackReturn:
    sn.logger.info("Deactivating...")
    sn.is_running = False
    return TransitionCallbackReturn.SUCCESS

def on_cleanup(sn: MySession) -> TransitionCallbackReturn:
    sn.logger.info("Cleaning up...")
    return TransitionCallbackReturn.SUCCESS

def on_shutdown(sn: MySession) -> None:
    sn.logger.info("Shutting down...")

if __name__ == "__main__":
    run(
        MySession,
        on_configure,
        on_activate=on_activate,
        on_deactivate=on_deactivate,
        on_cleanup=on_cleanup,
        on_shutdown=on_shutdown,
    )
```

## Automated Build System

### `cake_auto_package()`

The `cake_auto_package()` macro eliminates the need for manual CMake configuration by following a simple convention-over-configuration approach.

#### What It Does

When you call `cake_auto_package()`, it:

1. **Scans the `nodes/` directory** for subdirectories containing `interface.yaml` files
2. **Auto-detects languages** by looking for `.cpp` or `.py` files in each node directory
3. **Generates code** for each node:
   - C++: Interface headers, parameter libraries, and component registration code
   - Python: Interface modules, parameter classes, and executable wrappers
4. **Builds C++ libraries** from all `.cpp` files in the `nodes/` directory
5. **Registers components** with naming convention `${PROJECT_NAME}::${NodeName}`
6. **Creates executables** for both C++ (via component registration) and Python (via runpy wrappers)
7. **Installs everything** to proper locations (headers, libraries, executables, Python packages)
8. **Auto-installs common directories** like `launch/` and `config/` if they exist

#### Directory Convention

```
my_package/
├── nodes/                    # Required: All nodes go here
│   ├── my_cpp_node/
│   │   ├── interface.yaml   # Required
│   │   └── my_cpp_node.hpp  # Implementation
│   │   └── my_cpp_node.cpp  # Implementation
│   └── my_py_node/
│       ├── interface.yaml   # Required
│       └── my_py_node.py    # Implementation
├── launch/                   # Optional: Auto-installed if exists
├── config/                   # Optional: Auto-installed if exists
├── interfaces/               # Optional: Package-level interface definitions
├── CMakeLists.txt
└── package.xml
```

#### Multiple Nodes in One Package

You can have multiple nodes (both C++ and Python) in a single package:

```
my_package/
├── nodes/
│   ├── driver_node/
│   │   ├── interface.yaml
│   │   └── driver_node.hpp
│   │   └── driver_node.cpp
│   ├── controller_node/
│   │   ├── interface.yaml
│   │   └── controller_node.hpp
│   │   └── controller_node.cpp
│   └── monitor_node/
│       ├── interface.yaml
│       └── monitor_node.py
└── ...
```

All nodes will be built and registered automatically.

#### Install Additional Directories

```cmake
# Install additional directories to share/
cake_auto_package(INSTALL_TO_SHARE
    maps
    rviz
)
```

#### Component Plugin Naming

C++ nodes are registered as rclcpp components with this naming pattern:
- Plugin class: `${PROJECT_NAME}::${NodeName}`
- Executable: `${NODE_NAME}` (snake_case)

Example: A node `my_node` in package `my_package` becomes:
- Plugin: `my_package::MyNode`
- Executable: `my_node`

### `cake_auto_interface_package()`

The `cake_auto_interface_package()` macro simplifies the creation of ROS 2 interface packages by automatically discovering and generating all message, service, and action definitions.

#### What It Does

When you call `cake_auto_interface_package()`, it:

1. **Finds ament_cmake_auto** and discovers all dependencies from `package.xml`
2. **Auto-discovers interface files** in standard ROS 2 directories:
   - `msg/*.msg` for message definitions
   - `srv/*.srv` for service definitions
   - `action/*.action` for action definitions
3. **Generates interfaces** using `rosidl_generate_interfaces()` with auto-detected dependencies
4. **Finalizes the package** with `ament_auto_package()`

#### How to Use

**CMakeLists.txt:**
```cmake
cmake_minimum_required(VERSION 3.22)
project(my_interfaces)

find_package(cake REQUIRED)
cake_auto_interface_package()
```

That's it! Just 2 lines of actual code. The macro handles everything else.

**package.xml:**
```xml
<?xml version="1.0"?>
<package format="3">
  <name>my_interfaces</name>
  <version>0.0.0</version>
  <description>My interface definitions</description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>Apache 2.0</license>

  <buildtool_depend>cake</buildtool_depend>

  <!-- msg/service/action dependencies go here (if required) -->
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>std_srvs</depend>

  <!-- important! this must be included in all interface package xmls -->
  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

**Directory structure:**
```
my_interfaces/
├── msg/
│   ├── MyMessage.msg
│   └── AnotherMessage.msg
├── srv/
│   └── MyService.srv
├── action/
│   └── MyAction.action
├── CMakeLists.txt
└── package.xml
```

#### What It Replaces

Traditional interface package CMakeLists.txt files require manual file listing and explicit dependency management:

```cmake
# Old way (8+ lines)
cmake_minimum_required(VERSION 3.22)
project(my_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(msg_files "msg/MyMessage.msg" "msg/AnotherMessage.msg")
set(srv_files "srv/MyService.srv")
set(action_files "action/MyAction.action")

rosidl_generate_interfaces(${PROJECT_NAME}
    ${msg_files}
    ${srv_files}
    ${action_files}
    DEPENDENCIES std_msgs geometry_msgs action_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
```

With `cake_auto_interface_package()`, this becomes just 2 lines. All dependencies are automatically discovered from `package.xml`, and all interface files are automatically found.


## Interface YAML Specification

The interface YAML file defines your node's ROS 2 interfaces.

### Schema Validation

Cake provides a YAML Schema for `interface.yaml` files, enabling IDE autocompletion and validation.

#### VS Code Setup

Add to your `.vscode/settings.json` (adjust the path to your workspace):

```json
{
  "yaml.schemas": {
    "<your_workspace>/install/cake/share/cake/schemas/interface.schema.yaml": ["**/interface.yaml"]
  }
}
```

Or add a modeline comment to individual files:

```yaml
# yaml-language-server: $schema=<your_workspace>/install/cake/share/cake/schemas/interface.schema.yaml
publishers:
    ...
```

### Node Metadata

The `node:` section is **optional**. When omitted, `name` and `package` default to the values provided by the build system (derived from the directory structure and CMake `PROJECT_NAME`).

```yaml
# Minimal: no node section needed, defaults from build system
publishers:
    - topic: /cmd_vel
      ...
```

You can explicitly provide `node:` to override the defaults:

```yaml
node:
    name: custom_node_name
    package: custom_package
```

For backward compatibility, `${THIS_NODE}` and `${THIS_PACKAGE}` placeholders are still supported but no longer needed:

```yaml
node:
    name: ${THIS_NODE}       # Equivalent to omitting name entirely
    package: ${THIS_PACKAGE} # Equivalent to omitting package entirely
```

### Parameters

Uses `generate_parameter_library` (https://github.com/PickNikRobotics/generate_parameter_library) syntax:

```yaml
parameters:
    my_param:
        type: double
        default_value: 1.0
        description: "Parameter description"
        validation:
            gt<>: [0.0]
```

### Publishers

```yaml
publishers:
    - topic: /cmd_vel
      type: geometry_msgs/msg/Twist
      qos:
        history: 10
        reliability: RELIABLE
```

QoS is required for all publishers. See [QoS Configuration](#qos-configuration) for details. Topic names support `${param:name}` substitution — see [Dynamic Topic/Service/Action Names](#dynamic-topicserviceaction-names).

### Subscribers

```yaml
subscribers:
    - topic: /odom
      type: nav_msgs/msg/Odometry
      qos:
        history: 5
        reliability: BEST_EFFORT
```

QoS is required for all subscribers. See [QoS Configuration](#qos-configuration) for details. Topic names support `${param:name}` substitution — see [Dynamic Topic/Service/Action Names](#dynamic-topicserviceaction-names).

### Services

```yaml
services:
    - name: my_service
      type: example_interfaces/srv/AddTwoInts
```

### Service Clients

```yaml
service_clients:
    - name: external_service
      type: std_srvs/srv/Trigger
```

### Action Servers

```yaml
actions:
    - name: navigate
      type: nav2_msgs/action/NavigateToPose
```

### Action Clients

```yaml
action_clients:
    - name: navigate
      type: nav2_msgs/action/NavigateToPose
```

All entity types (services, service clients, actions, action clients) support `${param:name}` substitution in their names — see [Dynamic Topic/Service/Action Names](#dynamic-topicserviceaction-names).

### Common Optional Fields

All interface types (publishers, subscribers, services, service_clients, actions, action_clients) support the following optional field:

```yaml
manually_created: false  # Set to true to completely exclude from code generation
```

When `manually_created: true`, Cake will completely skip this interface during code generation - it won't appear in the generated session struct at all. This is useful when you want to document an interface in the YAML without having Cake generate code for it.

**Example:**
```yaml
subscribers:
    - topic: /camera/image
      type: sensor_msgs/msg/Image
      qos:
        history: 5
        reliability: BEST_EFFORT
      manually_created: true  # Won't be generated - handle this yourself
```

## QoS Configuration

QoS (Quality of Service) is **required** for all publishers and subscribers. QoS is not applicable to services, service clients, actions, or action clients.

### QoS Fields

**Required fields:**

| Field | Type | Values |
|-------|------|--------|
| `history` | integer or string | Integer > 0 for KEEP_LAST(n), or `"ALL"` for KEEP_ALL |
| `reliability` | string | `BEST_EFFORT` or `RELIABLE` |

**Optional fields:**

| Field | Type | Values |
|-------|------|--------|
| `durability` | string | `TRANSIENT_LOCAL` or `VOLATILE` |
| `deadline_ms` | integer | >= 0 (milliseconds) |
| `lifespan_ms` | integer | >= 0 (milliseconds) |
| `liveliness` | string | `AUTOMATIC` or `MANUAL_BY_TOPIC` |
| `lease_duration_ms` | integer | >= 0 (milliseconds, used with liveliness) |

### Examples

**Minimal QoS (required fields only):**
```yaml
qos:
  history: 10
  reliability: RELIABLE
```

**Sensor data (best effort, small queue):**
```yaml
qos:
  history: 5
  reliability: BEST_EFFORT
```

**Latched topic (transient local durability):**
```yaml
qos:
  history: 1
  reliability: RELIABLE
  durability: TRANSIENT_LOCAL
```

**With deadline monitoring:**
```yaml
qos:
  history: 10
  reliability: RELIABLE
  deadline_ms: 1000  # 1 second deadline
```

**Keep all messages:**
```yaml
qos:
  history: ALL
  reliability: RELIABLE
```

**Full configuration with all options:**
```yaml
qos:
  history: 5
  reliability: BEST_EFFORT
  durability: VOLATILE
  deadline_ms: 100
  lifespan_ms: 500
  liveliness: AUTOMATIC
  lease_duration_ms: 200
```

### QoS Parameter Substitution

QoS fields can reference `read_only` parameters using `${param:parameter_name}` syntax, allowing QoS settings to be configured at launch time rather than hardcoded.

**Requirements:**
- The referenced parameter must exist in the `parameters` section
- The parameter must have `read_only: true`
- The parameter type must be compatible with the QoS field:
  - `history`, `deadline_ms`, `lifespan_ms`, `lease_duration_ms`: requires `int` type
  - `reliability`, `durability`, `liveliness`: requires `string` type

**Example:**
```yaml
parameters:
  sensor_queue_depth:
    type: int
    default_value: 20
    read_only: true
    description: Queue depth for sensor data
  sensor_reliability:
    type: string
    default_value: RELIABLE
    read_only: true
    description: Reliability policy for sensor data

subscribers:
  - topic: /sensor_data
    type: sensor_msgs/msg/LaserScan
    qos:
      history: ${param:sensor_queue_depth}
      reliability: ${param:sensor_reliability}

publishers:
  - topic: /processed_data
    type: std_msgs/msg/String
    qos:
      history: ${param:sensor_queue_depth}
      reliability: RELIABLE  # Can mix literal values and param refs
```

You can then override QoS settings at launch time:
```bash
ros2 run my_package my_node --ros-args -p sensor_queue_depth:=50 -p sensor_reliability:=BEST_EFFORT
```

Or in a launch file:
```python
Node(
    package='my_package',
    executable='my_node',
    parameters=[{
        'sensor_queue_depth': 50,
        'sensor_reliability': 'BEST_EFFORT',
    }]
)
```

**Validation:** Invalid parameter values (e.g., `"INVALID"` for reliability) will raise an exception at node startup with a clear error message.

### Dynamic Topic/Service/Action Names

Topic, service, and action names can contain `${param:parameter_name}` references for dynamic name construction at startup. This is useful for multi-robot systems or configurable namespacing.

**Requirements:**
- The referenced parameter must exist in the `parameters` section
- The parameter must have `read_only: true`
- The parameter type must be `string` or `int`
- A `field_name` must be provided (since the topic name can't be used to derive a C++ identifier)

**Example:**
```yaml
parameters:
  robot_id:
    type: string
    default_value: "robot1"
    read_only: true
    description: "Robot identifier for topic namespacing"

publishers:
  - topic: /robot/${param:robot_id}/cmd_vel
    field_name: cmd_vel
    type: geometry_msgs/msg/Twist
    qos:
      history: 10
      reliability: RELIABLE

subscribers:
  - topic: /robot/${param:robot_id}/odom
    field_name: odom
    type: nav_msgs/msg/Odometry
    qos:
      history: 5
      reliability: BEST_EFFORT

services:
  - name: /robot/${param:robot_id}/get_state
    field_name: get_state
    type: std_srvs/srv/Trigger
```

This generates code that constructs the topic name at startup using the parameter value. In C++:
```cpp
// Generated: topic name built from parameter
cake::create_publisher<...>(sn, "/robot/" + cake::to_string(sn->params.robot_id) + "/cmd_vel", ...);
```

In Python:
```python
# Generated: topic name built from parameter
sn.publishers.cmd_vel._initialise(sn, Twist, f"/robot/{params.robot_id}/cmd_vel", ...)
```

You can then configure different robots at launch time:
```bash
ros2 run my_package my_node --ros-args -p robot_id:=robot2
```

**Multiple substitutions** are supported in a single name:
```yaml
subscribers:
  - topic: /${param:namespace}/${param:robot_id}/sensor
    field_name: sensor
    type: sensor_msgs/msg/Imu
    qos:
      history: 10
      reliability: RELIABLE
```

**Integer parameters** also work (converted to string automatically):
```yaml
parameters:
  sensor_num:
    type: int
    default_value: 1
    read_only: true

publishers:
  - topic: /sensor_${param:sensor_num}/data
    field_name: sensor_data
    type: std_msgs/msg/String
    qos:
      history: 10
      reliability: RELIABLE
```

**`field_name` explained:** When a topic/service/action name contains `${param:...}`, Cake can't derive a valid C++ field name from it automatically, so you must provide one explicitly. The `field_name` is used as the struct member name in the generated session:

```cpp
// With field_name: cmd_vel
sn->publishers.cmd_vel->publish(msg);
```

```python
# With field_name: cmd_vel
sn.publishers.cmd_vel.publish(msg)
```

The `field_name` property is also available for entities without parameter substitution, allowing you to override the auto-derived field name if desired.

### Dynamic Collections with `${for_each_param:...}`

When the **number** of entities varies per deployment (e.g., a lifecycle manager that needs N service clients for N managed nodes), use `${for_each_param:parameter_name}` in entity names. This generates a `std::unordered_map` (C++) or `dict` (Python) keyed by the parameter's string values, with a loop that creates one entity per element at startup.

**Requirements:**
- The referenced parameter must exist in the `parameters` section
- The parameter must have `read_only: true`
- The parameter type must be `string_array`
- A `field_name` must be provided
- Only **one** `${for_each_param:...}` reference is allowed per entity name (but `${param:...}` references can coexist alongside it)

**Example:**
```yaml
parameters:
  managed_nodes:
    type: string_array
    default_value:
      - "node_a"
      - "node_b"
    read_only: true
    description: "List of managed node names"
  robot_id:
    type: string
    default_value: "robot1"
    read_only: true
    description: "Robot identifier"

publishers:
  # Regular publisher — single instance, uses ${param:...}
  - topic: /robot/${param:robot_id}/status
    field_name: status
    type: std_msgs/msg/String
    qos:
      history: 10
      reliability: RELIABLE

subscribers:
  # for_each_param subscriber — one per managed node
  - topic: /${for_each_param:managed_nodes}/state
    field_name: node_states
    type: std_msgs/msg/String
    qos:
      history: 10
      reliability: RELIABLE

service_clients:
  # for_each_param service client — one per managed node
  - name: /${for_each_param:managed_nodes}/change_state
    field_name: change_state_clients
    type: lifecycle_msgs/srv/ChangeState
```

This generates map/dict-typed fields and loop initialization. In C++:

```cpp
// Struct field is a map
std::unordered_map<std::string, std::shared_ptr<cake::Subscriber<std_msgs::msg::String, SessionType>>> node_states;

// Constructor loops over parameter values
for (const auto& key : sn->params.managed_nodes) {
    sn->subscribers.node_states[key] = cake::create_subscriber<std_msgs::msg::String>(
        sn, "/" + key + "/state", rclcpp::QoS(10).reliable());
}
```

In Python:

```python
# Dataclass field is a dict
node_states: dict[str, cake.Subscriber[String]] = field(default_factory=dict)

# Initialization loops over parameter values
for key in params.managed_nodes:
    sn.subscribers.node_states[key] = cake.Subscriber[String]()
    sn.subscribers.node_states[key]._initialise(sn, String, f"/{key}/state", ...)
```

Access entities by iterating over the map/dict at runtime:

```cpp
// C++
for (const auto& [name, client] : sn->service_clients.change_state_clients) {
    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    client->async_send_request(request);
}
```

```python
# Python
for name, client in sn.service_clients.change_state_clients.items():
    request = ChangeState.Request()
    client.call_async(request)
```

`${for_each_param:...}` works with all entity types: publishers, subscribers, services, service clients, actions, and action clients.

## QoS Event Callbacks

Cake subscribers and publishers support QoS event callbacks to react when deadlines are missed or liveliness changes.

### Deadline Callback

The deadline callback fires when no message is received within the deadline period specified in QoS:

**C++ Example:**
```cpp
CallbackReturn on_configure(std::shared_ptr<Session> sn) {
    // Set the message callback
    sn->subscribers.ok->set_callback(
        [](std::shared_ptr<Session> sn, std_msgs::msg::Bool::ConstSharedPtr msg) {
            sn->ok_received = true;
            sn->ok_status = msg->data;
        }
    );

    // Set deadline callback - fires when no message received in time
    sn->subscribers.ok->set_deadline_callback(
        [](std::shared_ptr<Session> sn, rclcpp::QOSDeadlineRequestedInfo& event) {
            RCLCPP_WARN(sn->node.get_logger(), "Deadline missed!");
            sn->ok_received = false;
        }
    );

    return CallbackReturn::SUCCESS;
}
```

**Python Example:**
```python
def on_configure(sn: MySession) -> TransitionCallbackReturn:
    def on_msg(sn, msg):
        sn.ok_received = True
        sn.ok_status = msg.data

    def on_deadline_missed(sn, event):
        sn.node.get_logger().warning("Deadline missed!")
        sn.ok_received = False

    sn.subscribers.ok.set_callback(on_msg)
    sn.subscribers.ok.set_deadline_callback(on_deadline_missed)

    return TransitionCallbackReturn.SUCCESS
```

### Subscriber Liveliness Callback

The liveliness callback fires when a publisher's liveliness state changes:

```cpp
sn->subscribers.sensor->set_liveliness_callback(
    [](std::shared_ptr<Session> sn, rclcpp::QOSLivelinessChangedInfo& event) {
        RCLCPP_INFO(sn->node.get_logger(),
            "Liveliness changed: %d alive, %d not alive",
            event.alive_count, event.not_alive_count);
    }
);
```

Subscribers also expose the underlying `rclcpp::Subscription` / `rclpy.subscription.Subscription` via the `subscription()` method for advanced use cases.

### Publisher QoS Callbacks

Publishers also support QoS event callbacks. Note the different event types compared to subscribers:

- **Subscriber deadline**: `QOSDeadlineRequestedInfo` - didn't receive message in time
- **Publisher deadline**: `QOSDeadlineOfferedInfo` - didn't publish in time
- **Subscriber liveliness**: `QOSLivelinessChangedInfo` - publisher liveliness changed
- **Publisher liveliness**: `QOSLivelinessLostInfo` - our liveliness was lost

**C++ Example:**
```cpp
CallbackReturn on_configure(std::shared_ptr<Session> sn) {
    // Deadline callback - fires when we don't publish in time
    sn->publishers.status->set_deadline_callback(
        [](std::shared_ptr<Session> sn, rclcpp::QOSDeadlineOfferedInfo& event) {
            RCLCPP_WARN(sn->node.get_logger(), "Missed publish deadline!");
        }
    );

    // Liveliness callback - fires when our liveliness is lost
    sn->publishers.status->set_liveliness_callback(
        [](std::shared_ptr<Session> sn, rclcpp::QOSLivelinessLostInfo& event) {
            RCLCPP_WARN(sn->node.get_logger(), "Liveliness lost!");
        }
    );

    return CallbackReturn::SUCCESS;
}
```

**Python Example:**
```python
def on_configure(sn: MySession) -> TransitionCallbackReturn:
    def on_deadline_missed(sn, event):
        sn.node.get_logger().warning("Missed publish deadline!")

    def on_liveliness_lost(sn, event):
        sn.node.get_logger().warning("Liveliness lost!")

    sn.publishers.status.set_deadline_callback(on_deadline_missed)
    sn.publishers.status.set_liveliness_callback(on_liveliness_lost)

    return TransitionCallbackReturn.SUCCESS
```

Publishers also expose the underlying `rclcpp::Publisher` / `rclpy.publisher.Publisher` via the `publisher()` method for advanced use cases like `wait_for_all_acked()` or `get_subscription_count()`.

### Configuring QoS for Event Callbacks

For deadline callbacks to work, you must set a deadline in your QoS configuration:

```yaml
subscribers:
    - topic: ok
      type: std_msgs/msg/Bool
      qos:
        history: 10
        reliability: RELIABLE
        deadline_ms: 1000  # 1 second

publishers:
    - topic: status
      type: std_msgs/msg/String
      qos:
        history: 10
        reliability: RELIABLE
        deadline_ms: 500  # 500ms
```

For liveliness callbacks, configure liveliness and lease duration:

```yaml
subscribers:
    - topic: sensor
      type: sensor_msgs/msg/Imu
      qos:
        history: 5
        reliability: BEST_EFFORT
        liveliness: AUTOMATIC
        lease_duration_ms: 2000  # 2 seconds

publishers:
    - topic: heartbeat
      type: std_msgs/msg/Empty
      qos:
        history: 1
        reliability: RELIABLE
        liveliness: AUTOMATIC
        lease_duration_ms: 1000  # 1 second
```

## Autostart

By default, Cake nodes **automatically transition through configure → activate** on startup, so they begin processing immediately without requiring an external lifecycle manager.

This is controlled by the `autostart` parameter (default: `true`). A zero-delay timer fires once on construction to call `trigger_configure()` followed by `trigger_activate()`. If either transition fails, an error is logged and the sequence stops.

To disable autostart (e.g., when using a lifecycle manager):

```bash
ros2 run my_package my_node --ros-args -p autostart:=false
```

Or in a launch file:

```python
Node(
    package='my_package',
    executable='my_node',
    parameters=[{'autostart': False}]
)
```

When `autostart` is disabled, you must trigger lifecycle transitions externally:

```bash
ros2 lifecycle set /my_node configure
ros2 lifecycle set /my_node activate
```

## State Heartbeat

Every Cake node publishes its current lifecycle state on `~/state` at 10 Hz. This provides a lightweight monitoring and watchdog interface without polling the lifecycle service.

| Property | Value |
|----------|-------|
| Topic | `~/state` |
| Type | `lifecycle_msgs/msg/State` |
| Rate | 100 ms |
| QoS | Reliable, transient-local, 100 ms deadline, automatic liveliness (100 ms lease) |

The publisher only serialises messages when there are active subscribers, so there is zero overhead when nobody is listening.

**Watchdog usage:** External monitors can subscribe with a matching deadline QoS. If the node hangs or crashes and stops publishing, the subscriber's deadline-missed or liveliness change callback fires, enabling automatic fault detection.

```bash
# Monitor a node's state from the command line
ros2 topic echo /my_node/state
```

## Intra-Process Communication

C++ Cake nodes enable **intra-process communication (IPC)** by default. When multiple Cake nodes run in the same process (e.g., via component composition), messages are passed by pointer rather than serialised, providing zero-copy performance.

This is set automatically in the `BaseNode` constructor — no configuration is needed.

> **Note:** IPC is a C++ feature. Python nodes are unaffected.

## Default QoS Handlers

Cake automatically attaches **default QoS event handlers** to every generated subscriber. These handlers provide a safety net that deactivates the node when QoS contracts are violated:

| Event | Behaviour |
|-------|-----------|
| **Deadline missed** | Logs an error and deactivates the node |
| **Liveliness changed** (alive publishers drops to 0) | Logs an error and deactivates the node |

Both handlers are no-ops when the node is not in the `ACTIVE` state, preventing spurious triggers during transitions.

This gives you **cascading shutdown** for free: if an upstream node deactivates and stops publishing, downstream subscribers miss their deadline (or lose liveliness) and automatically deactivate too.

### How It Works

For every subscriber defined in `interface.yaml`, the generated code calls `attach_default_qos_handlers()` immediately after creation. No user code is needed — the handlers are always present.

To make the handlers trigger, configure a `deadline_ms` and/or `liveliness` + `lease_duration_ms` in your subscriber's QoS:

```yaml
subscribers:
    - topic: heartbeat
      type: std_msgs/msg/Bool
      qos:
        history: 1
        reliability: RELIABLE
        deadline_ms: 1000           # deactivate if no message for 1s
        liveliness: AUTOMATIC
        lease_duration_ms: 1000     # deactivate if publisher disappears
```

Without deadline or liveliness QoS settings, the handlers are attached but will never fire.

### Overriding Default Handlers

The default handlers call `set_deadline_callback()` and `set_liveliness_callback()` on the subscriber. If you set your own callbacks in `on_configure`, they will **replace** the defaults:

```cpp
// Override the default deadline handler with custom logic
sn->subscribers.heartbeat->set_deadline_callback(
    [](std::shared_ptr<Session> sn, rclcpp::QOSDeadlineRequestedInfo& event) {
        RCLCPP_WARN(sn->node.get_logger(), "Custom deadline handling");
        // your logic here
    }
);
```

### Using Default Handlers Manually

You can also attach the default handlers to manually-created subscribers:

**C++:**
```cpp
#include <cake/default_qos_handlers.hpp>

// After creating a subscriber manually
cake::attach_default_qos_handlers(sn->subscribers.my_sub);
```

**Python:**
```python
import cake

cake.attach_default_qos_handlers(sn.subscribers.my_sub)
```

## Threading Model

Cake assumes a **single-threaded executor** for most callbacks. Session state (publishers, subscribers, parameters, timers, etc.) is not protected by any synchronization primitives, so concurrent access from multiple executor threads would be a data race. Multi-threading executors is out of scope for cake at the moment. External concurrent execution of work is still available to the user via standard threading, but synchronisation is the users responsibility.

### Isolated Callback Group for Service/Action Clients

Service clients and action clients are placed on a **dedicated callback group** with its own background `SingleThreadedExecutor` thread. This means their response callbacks are processed independently of the main executor — preventing deadlocks when calling services synchronously from lifecycle callbacks (e.g., `on_configure`).

Without this, calling a service synchronously from `on_configure` would deadlock: the main executor thread is blocked waiting for the response, but that same thread is the only one that can process the response.

The background executor is created in the `BaseNode` constructor and torn down in the destructor. Generated code automatically passes the isolated callback group when creating service and action clients — no user configuration is needed.

## Synchronous Service & Action Helpers (C++)

Cake provides `<cake/call_sync.hpp>` with blocking wrappers for service and action clients. These are safe to call from lifecycle callbacks because the isolated background executor processes the responses.

### `cake::call_sync` — Synchronous Service Call

```cpp
#include <cake/call_sync.hpp>

CallbackReturn on_configure(std::shared_ptr<Session> sn) {
    if (sn->service_clients.my_service->wait_for_service(2s)) {
        auto req = std::make_shared<MyService::Request>();
        req->data = 42;
        auto resp = cake::call_sync<MyService>(sn->service_clients.my_service, req, 5s);
        if (resp) {
            RCLCPP_INFO(sn->node.get_logger(), "Got response: %d", resp->result);
        } else {
            RCLCPP_WARN(sn->node.get_logger(), "Service call timed out");
        }
    }
    return CallbackReturn::SUCCESS;
}
```

Returns `nullptr` on timeout. Default timeout is 5 seconds.

### `cake::send_goal_sync` — Synchronous Action Goal

```cpp
#include <cake/call_sync.hpp>

auto goal = MyAction::Goal();
goal.order = 5;
auto goal_handle = cake::send_goal_sync<MyAction>(
    sn->action_clients.my_action, goal, {}, 5s);
if (goal_handle) {
    RCLCPP_INFO(sn->node.get_logger(), "Goal accepted");
}
```

Returns `nullptr` if the goal is rejected or the request times out.

### `cake::get_result_sync` — Wait for Action Result

```cpp
auto result = cake::get_result_sync<MyAction>(
    sn->action_clients.my_action, goal_handle, 5min);
if (result) {
    RCLCPP_INFO(sn->node.get_logger(), "Result: %d", result->sequence.size());
}
```

Returns `std::nullopt` on timeout. Default timeout is 5 minutes.

### `cake::cancel_goal_sync` — Synchronous Goal Cancel

```cpp
auto cancel_resp = cake::cancel_goal_sync<MyAction>(
    sn->action_clients.my_action, goal_handle, 5s);
```

Returns `nullptr` on timeout.

> **Important:** These helpers are designed for calling services/actions on **other nodes**. Calling a service hosted on the same node from a callback on the main executor will still deadlock, because the service handler also needs the main executor thread to run.

## Development

### Running Tests

```bash
cd cake/tests
./run_tests.sh
```

### Accepting Test Outputs

After making changes to the code generator:

```bash
cd cake/tests
./run_tests.sh         # Generate new outputs
./accept_outputs.sh    # Accept as expected outputs
./run_tests.sh         # Verify tests pass
```

## Examples

The `cake_example` package demonstrates usage with:
- **Multiple nodes**: C++ node (`my_node`) and Python node (`python_node`)
- **Interface examples**: Publishers, subscribers, services, actions, and parameters
- **Synchronous calls**: `my_node` uses `cake::call_sync` and `cake::send_goal_sync` in `on_configure` to call `python_node`'s service and action synchronously — demonstrating deadlock-free sync calls from lifecycle callbacks
- **Cascading deactivation**: `my_node` publishes a heartbeat; `python_node` subscribes with a 1 s deadline — if `my_node` deactivates, the default QoS handler automatically deactivates `python_node` too
- **Minimal CMakeLists.txt**: Just 3 lines using `cake_auto_package()`
- **Component registration**: Automatic component plugin setup
- **Package-level interfaces**: Optional `interfaces/` directory for shared definitions

Structure:
```
cake_example/
├── nodes/
│   ├── my_node/
│   │   ├── interface.yaml
│   │   ├── my_node.cpp
│   │   └── my_node.hpp
│   └── python_node/
│       ├── interface.yaml
│       └── python_node.py
├── interfaces/
│   ├── external_node.yaml
│   └── transition_node.yaml
├── launch/
│   └── test.launch.py
├── CMakeLists.txt          # Just cake_auto_package()!
└── package.xml
```

Build and run the example:

```bash
colcon build --packages-select cake_example
source install/setup.bash

# Run C++ node
ros2 run cake_example my_node

# Run Python node
ros2 run cake_example python_node

# Load as component
ros2 component standalone cake_example cake_example::MyNode
```

## License

Licensed under the Apache License, Version 2.0. See [LICENSE](LICENSE) for details.
