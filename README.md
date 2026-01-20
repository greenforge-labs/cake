# cake

**Declarative code generation for ROS 2 nodes**

Cake transforms simple YAML interface definitions into strongly-typed C++ and Python ROS 2 node scaffolding. Define your publishers, subscribers, services, actions and parameters in one simple file and Cake will handle the rest.

Using cake, writing ROS2 nodes becomes a...  <sub>piece of cake (hehe)</sub>

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
node:
    name: ${THIS_NODE}
    package: ${THIS_PACKAGE}

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

> **Design Pattern:** Cake stores the mutable state of a node in a `Context` class rather than subclassing `rclcpp::Node`. This separation makes testing easier (you can test logic without spinning up ROS), keeps state explicit, and allows callbacks to be simple free functions. To define the `Context` of your node, you subclass the auto-generated `<NodeName>Context` struct and add your own variables to it. The auto-generated `Context` class will contain a pointer to your ROS2 node instance, as well as all publishers, subscribers, services, actions and parameters.

```cpp
#pragma once

#include <memory>
#include <my_package/my_node_interface.hpp>

namespace my_package::my_node {

// Extend the generated context with custom state
struct Context : MyNodeContext<Context> {
    // Add any custom state here
    int my_counter = 0;
};

// Forward declare init function
void init(std::shared_ptr<Context> ctx);

// Define the node class using the generated base
// This must match the pattern: package::node_name::NodeName
using MyNode = MyNodeBase<Context, init>;

} // namespace my_package::my_node
```

Then implement it (`nodes/my_node/my_node.cpp`):

> **Design Pattern:** Cake uses a functional `init()` approach instead of subclassing `rclcpp::Node` with constructors. The `init()` function receives a fully-constructed context with all publishers, subscribers, and parameters ready to use. This functional approach, coupled with the context object, makes nodes easier to reason about, simpler to write and more testable. By putting a pointer to the ROS node in the context, we create a "has-a" relationship with the Node rather than "is-a", cleanly separating ROS communication from your implementation logic.

```cpp
#include "my_node.hpp"

namespace my_package::my_node {

void msg_callback(std::shared_ptr<Context> ctx, std_msgs::msg::Bool::ConstSharedPtr msg) {
    ctx->my_counter++;
    RCLCPP_INFO(ctx->node->get_logger(), "Got a bool: %d (count: %d)", msg->data, ctx->my_counter);
}

void addition_request_handler(
    std::shared_ptr<Context> ctx,
    example_interfaces::srv::AddTwoInts::Request::SharedPtr request,
    example_interfaces::srv::AddTwoInts::Response::SharedPtr response
) {
    response->sum = request->a + request->b;
}

void init(std::shared_ptr<Context> ctx) {
    // Access parameters
    RCLCPP_INFO(ctx->node->get_logger(), "important_parameter: %s", ctx->params.important_parameter.c_str());

    // Publish messages
    auto msg = std_msgs::msg::String();
    msg.data = ctx->params.important_parameter;
    ctx->publishers.some_topic->publish(msg);

    // Set callbacks
    ctx->subscribers.other_topic->set_callback(msg_callback);
    ctx->services.my_service->set_request_handler(addition_request_handler);
}

} // namespace my_package::my_node
```

#### Python Example (`nodes/my_node/my_node.py`)

```python
from dataclasses import dataclass

from my_package.my_node import MyNodeContext, run
from std_msgs.msg import String

# Extend the generated context with custom state
@dataclass
class Context(MyNodeContext):
    my_counter: int = 0

def msg_callback(ctx: Context, msg):
    ctx.my_counter += 1
    ctx.logger.info(f"Got a bool: {msg.data} (count: {ctx.my_counter})")

def init(ctx: Context):
    # Access parameters
    ctx.logger.info(f"important_parameter: {ctx.params.important_parameter}")

    # Publish messages
    msg = String()
    msg.data = ctx.params.important_parameter
    ctx.publishers.some_topic.publish(msg)

    # Set callbacks
    ctx.subscribers.other_topic.set_callback(msg_callback)

if __name__ == "__main__":
    run(Context, init)
```

### 4. Create CMakeLists.txt

This is all you need in your `CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.22)
project(my_package)

find_package(cake REQUIRED)
cake_auto_package()
```

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
node:
    name: ${THIS_NODE}
    ...
```

### Node Metadata

```yaml
node:
    name: ${THIS_NODE}       # Replaced by the node name detected from the folder structure
    package: ${THIS_PACKAGE} # Replaced by the package name detected from cmake project name
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

QoS is required for all publishers. See [QoS Configuration](#qos-configuration) for details.

### Subscribers

```yaml
subscribers:
    - topic: /odom
      type: nav_msgs/msg/Odometry
      qos:
        history: 5
        reliability: BEST_EFFORT
```

QoS is required for all subscribers. See [QoS Configuration](#qos-configuration) for details.

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

### Common Optional Fields

All interface types (publishers, subscribers, services, service_clients, actions, action_clients) support the following optional field:

```yaml
manually_created: false  # Set to true to completely exclude from code generation
```

When `manually_created: true`, Cake will completely skip this interface during code generation - it won't appear in the generated context struct at all. This is useful when you want to document an interface in the YAML without having Cake generate code for it.

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

## QoS Event Callbacks

Cake subscribers and publishers support QoS event callbacks to react when deadlines are missed or liveliness changes.

### Deadline Callback

The deadline callback fires when no message is received within the deadline period specified in QoS:

**C++ Example:**
```cpp
void init(std::shared_ptr<Context> ctx) {
    // Set the message callback
    ctx->subscribers.ok->set_callback(
        [](std::shared_ptr<Context> ctx, std_msgs::msg::Bool::ConstSharedPtr msg) {
            ctx->ok_received = true;
            ctx->ok_status = msg->data;
        }
    );

    // Set deadline callback - fires when no message received in time
    ctx->subscribers.ok->set_deadline_callback(
        [](std::shared_ptr<Context> ctx, rclcpp::QOSDeadlineRequestedInfo& event) {
            RCLCPP_WARN(ctx->node->get_logger(), "Deadline missed!");
            ctx->ok_received = false;
        }
    );
}
```

**Python Example:**
```python
def init(ctx: Context):
    def on_msg(ctx, msg):
        ctx.ok_received = True
        ctx.ok_status = msg.data

    def on_deadline_missed(ctx, event):
        ctx.node.get_logger().warning("Deadline missed!")
        ctx.ok_received = False

    ctx.subscribers.ok.set_callback(on_msg)
    ctx.subscribers.ok.set_deadline_callback(on_deadline_missed)
```

### Subscriber Liveliness Callback

The liveliness callback fires when a publisher's liveliness state changes:

```cpp
ctx->subscribers.sensor->set_liveliness_callback(
    [](std::shared_ptr<Context> ctx, rclcpp::QOSLivelinessChangedInfo& event) {
        RCLCPP_INFO(ctx->node->get_logger(),
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
void init(std::shared_ptr<Context> ctx) {
    // Deadline callback - fires when we don't publish in time
    ctx->publishers.status->set_deadline_callback(
        [](std::shared_ptr<Context> ctx, rclcpp::QOSDeadlineOfferedInfo& event) {
            RCLCPP_WARN(ctx->node->get_logger(), "Missed publish deadline!");
        }
    );

    // Liveliness callback - fires when our liveliness is lost
    ctx->publishers.status->set_liveliness_callback(
        [](std::shared_ptr<Context> ctx, rclcpp::QOSLivelinessLostInfo& event) {
            RCLCPP_WARN(ctx->node->get_logger(), "Liveliness lost!");
        }
    );
}
```

**Python Example:**
```python
def init(ctx: Context):
    def on_deadline_missed(ctx, event):
        ctx.node.get_logger().warning("Missed publish deadline!")

    def on_liveliness_lost(ctx, event):
        ctx.node.get_logger().warning("Liveliness lost!")

    ctx.publishers.status.set_deadline_callback(on_deadline_missed)
    ctx.publishers.status.set_liveliness_callback(on_liveliness_lost)
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
