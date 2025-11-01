# cake

A collection of tools to make ROS2 development easier.

## Overview

Cake provides code generation tools that reduce boilerplate and make ROS2 node development more maintainable:

- **Node Interface Generator**: Automatically generate C++ node interfaces from YAML definitions
- **Type-safe Context**: Generated context structures with compile-time type checking
- **Declarative ROS2 Interfaces**: Define publishers, subscribers, services, service clients, and action servers in YAML instead of C++
- **Flexible QoS Configuration**: Support for predefined profiles and custom parameters
- **Polling-based Action Servers**: Simple action server implementation designed for main-loop processing

## Quick Start

### 1. Define Your Node Interface

Create an `interface.yaml` file:

```yaml
node:
    name: my_node
    package: ${THIS_PACKAGE}

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

### 2. Add to CMakeLists.txt

```cmake
find_package(cake REQUIRED)

cake_generate_node_interface(my_node_interface nodes/my_node/interface.yaml)

add_executable(my_node src/my_node.cpp)
target_link_libraries(my_node my_node_interface)
```

### 3. Use the Generated Interface

```cpp
#include <my_package/my_node_interface.hpp>

struct MyContext : my_package::my_node::MyNodeContext<MyContext> {};

void init(std::shared_ptr<MyContext> ctx) {
    // Your initialization code
    // ctx->publishers.cmd_vel is ready to use
    // ctx->subscribers.odom is ready to use

    // Set up service handler
    ctx->services.reset->set_request_handler(
        [](auto ctx, auto request, auto response) {
            response->success = true;
        }
    );
}

using MyNode = my_package::my_node::MyNodeBase<MyContext, init>;

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
    name: my_node              # Node name (required)
    package: ${THIS_PACKAGE}   # Package name or ${THIS_PACKAGE} for auto-detection
```

The `${THIS_PACKAGE}` placeholder is automatically replaced with the package name from CMake's `PROJECT_NAME`.

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

### Action Servers

```yaml
action_servers:
    - name: fibonacci                           # Action name (required)
      type: example_interfaces/action/Fibonacci # Action type (required)
      manually_created: false                   # Skip auto-generation (optional, default: false)
```

Action servers use the `SingleGoalActionServer` which manages goal lifecycle and ensures only one goal is active at a time. The design is polling-based - you check for active goals in your main loop rather than using callbacks.

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

    ctx->action_servers.fibonacci->set_options(options);
}
```

**Note:** Action servers will reject all goals until options are set via `set_options()`.

#### Processing Goals (Polling Pattern)

Check for active goals in your node's main loop or timer callbacks:

```cpp
cake::create_timer(ctx, 100ms, [](auto ctx) {
    auto goal = ctx->action_servers.fibonacci->get_active_goal();
    if (!goal) {
        return;  // No active goal (or it was cancelled)
    }

    // Process goal incrementally
    auto feedback = std::make_shared<example_interfaces::action::Fibonacci::Feedback>();
    feedback->sequence.push_back(current_value);
    ctx->action_servers.fibonacci->publish_feedback(feedback);

    // Complete when done
    if (is_complete) {
        auto result = std::make_shared<example_interfaces::action::Fibonacci::Result>();
        result->sequence = final_sequence;
        ctx->action_servers.fibonacci->succeed(result);
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
