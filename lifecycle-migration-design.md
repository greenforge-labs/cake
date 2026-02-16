# Cake Lifecycle Node Migration — Design Document

## Overview

Replace `rclcpp::Node` with `rclcpp_lifecycle::LifecycleNode` throughout the cake framework. This is a breaking change. After migration, all cake nodes are lifecycle nodes with the standard state machine: **Unconfigured -> Inactive -> Active -> Finalized**.

### Key decisions

- **User callback API**: Template parameters. `on_configure` is required (replaces `init`), `on_activate` / `on_deactivate` / `on_cleanup` are optional with no-op defaults.
- **Subscribers**: Auto-drop messages when node is not Active.
- **Services**: Auto-reject (default-constructed response) when node is not Active.
- **Action servers**: Auto-reject goals when node is not Active.
- **Timers**: Created inactive when node is not Active, callback-guarded for safety.
- **Publishers**: Use `LifecyclePublisher` — publishing is a no-op when not Active. Generated code handles activate/deactivate.
- **BaseNode**: Direct inheritance from `LifecycleNode` (no composition, no `LifecycleNodeImpl` wrapper). The move from constructor-based init to `on_configure` makes `shared_from_this()` available, which was the original blocker for inheritance.
- **Only lifecycle nodes**: No flag/option to use regular `rclcpp::Node`.
- **Weak pointers in wrappers**: Entity wrappers (Publisher, Subscriber, Service) and timer/thread callbacks store `weak_ptr<Context>` instead of `shared_ptr<Context>`. This eliminates reference cycles between the context and its entities, making cleanup trivial (`ctx_.reset()` frees everything) and ensuring callbacks become graceful no-ops if the context is destroyed.

---

## Phase 1: Core headers — `base_node.hpp` and `context.hpp`

These are the foundation. Everything else depends on them.

### `context.hpp`

Change the node type and add the lifecycle header:

```cpp
#pragma once

#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace cake {

struct Context {
    rclcpp_lifecycle::LifecycleNode::SharedPtr node;
    std::vector<rclcpp::TimerBase::SharedPtr> timers;
};

} // namespace cake
```

### `base_node.hpp`

Switch from composition to inheritance. `BaseNode` IS a `LifecycleNode`:

```cpp
#pragma once

#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "fixed_string.hpp"

namespace cake {

template <fixed_string node_name, auto extend_options = [](rclcpp::NodeOptions options) { return options; }>
class BaseNode : public rclcpp_lifecycle::LifecycleNode {
  public:
    explicit BaseNode(const rclcpp::NodeOptions &options)
        : LifecycleNode(node_name.c_str(), extend_options(options)) {}
};

} // namespace cake
```

Notes:
- `get_node_base_interface()` is inherited from `LifecycleNode` — no need to implement it.
- `RCLCPP_COMPONENTS_REGISTER_NODE` continues to work since `LifecycleNode` provides the required interface.
- The `protected: rclcpp::Node::SharedPtr node_` member is gone. The generated `*Base` class uses `this->shared_from_this()` in `on_configure` to populate `ctx->node`.

---

## Phase 2: Entity wrappers

### `publisher.hpp`

Change the underlying publisher type to `LifecyclePublisher`. Add `activate()` / `deactivate()` methods called by generated lifecycle code. Store `weak_ptr<ContextType>` to avoid reference cycles.

```cpp
#pragma once

#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

namespace cake {

template <typename MessageT, typename ContextType> class Publisher {
  public:
    explicit Publisher(std::shared_ptr<ContextType> context, const std::string &topic_name, const rclcpp::QoS &qos)
        : context_(context) {
        rclcpp::PublisherOptions options;
        options.event_callbacks.deadline_callback = [this](rclcpp::QOSDeadlineOfferedInfo &event) {
            if (auto ctx = context_.lock()) {
                if (deadline_callback_) {
                    deadline_callback_(ctx, event);
                }
            }
        };
        options.event_callbacks.liveliness_callback = [this](rclcpp::QOSLivelinessLostInfo &event) {
            if (auto ctx = context_.lock()) {
                if (liveliness_callback_) {
                    liveliness_callback_(ctx, event);
                }
            }
        };

        publisher_ = context->node->template create_publisher<MessageT>(topic_name, qos, options);
    }

    void publish(const MessageT &msg) { publisher_->publish(msg); }
    void publish(std::unique_ptr<MessageT> msg) { publisher_->publish(std::move(msg)); }

    void activate() { publisher_->on_activate(); }
    void deactivate() { publisher_->on_deactivate(); }

    typename rclcpp_lifecycle::LifecyclePublisher<MessageT>::SharedPtr publisher() { return publisher_; }

    void set_deadline_callback(
        std::function<void(std::shared_ptr<ContextType>, rclcpp::QOSDeadlineOfferedInfo &)> callback
    ) {
        deadline_callback_ = callback;
    }

    void set_liveliness_callback(
        std::function<void(std::shared_ptr<ContextType>, rclcpp::QOSLivelinessLostInfo &)> callback
    ) {
        liveliness_callback_ = callback;
    }

  private:
    std::weak_ptr<ContextType> context_;
    typename rclcpp_lifecycle::LifecyclePublisher<MessageT>::SharedPtr publisher_;

    std::function<void(std::shared_ptr<ContextType>, rclcpp::QOSDeadlineOfferedInfo &)> deadline_callback_;
    std::function<void(std::shared_ptr<ContextType>, rclcpp::QOSLivelinessLostInfo &)> liveliness_callback_;
};

template <typename MessageT, typename ContextType>
std::shared_ptr<Publisher<MessageT, ContextType>>
create_publisher(std::shared_ptr<ContextType> context, const std::string &topic_name, const rclcpp::QoS &qos) {
    return std::make_shared<Publisher<MessageT, ContextType>>(context, topic_name, qos);
}

} // namespace cake
```

Notes:
- `LifecycleNode::create_publisher` returns `LifecyclePublisher`, which silently drops `publish()` calls when deactivated.
- `activate()` / `deactivate()` are called by the generated `on_activate` / `on_deactivate` — not by user code.
- The `publisher()` accessor return type changes from `rclcpp::Publisher<MessageT>::SharedPtr` to `rclcpp_lifecycle::LifecyclePublisher<MessageT>::SharedPtr`. This is a breaking change for anyone calling `publisher()` directly.
- `context_` is `weak_ptr` — no reference cycle. QoS event callbacks lock before use; if the context is gone, the callback is a no-op.

### `subscriber.hpp`

Add lifecycle state check to the callback wrapper. Messages received when the node is not Active are silently dropped. Store `weak_ptr<ContextType>` to avoid reference cycles.

The change is in the `context_` member type (from `shared_ptr` to `weak_ptr`) and the callback lambdas (lock before use):

```cpp
template <typename MessageT, typename ContextType> class Subscriber {
  public:
    explicit Subscriber(std::shared_ptr<ContextType> context, const std::string &topic_name, const rclcpp::QoS &qos)
        : context_(context) {
        // ... default callback setup unchanged ...

        rclcpp::SubscriptionOptions options;
        options.event_callbacks.deadline_callback = [this](rclcpp::QOSDeadlineRequestedInfo &event) {
            if (auto ctx = context_.lock()) {
                if (deadline_callback_) {
                    deadline_callback_(ctx, event);
                }
            }
        };
        options.event_callbacks.liveliness_callback = [this](rclcpp::QOSLivelinessChangedInfo &event) {
            if (auto ctx = context_.lock()) {
                if (liveliness_callback_) {
                    liveliness_callback_(ctx, event);
                }
            }
        };

        subscription_ = context->node->template create_subscription<MessageT>(
            topic_name, qos,
            [this](typename MessageT::ConstSharedPtr msg) {
                auto ctx = context_.lock();
                if (!ctx) return;
                if (ctx->node->get_current_state().id() ==
                    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
                    callback_(ctx, msg);
                }
            }, options);
    }

    // set_callback signature unchanged — still takes std::function with shared_ptr<ContextType>

  private:
    std::weak_ptr<ContextType> context_;
    // ... rest unchanged ...
};
```

Requires `#include <lifecycle_msgs/msg/state.hpp>`.

### `service.hpp`

Add lifecycle state check. When not Active, log a warning and return a default-constructed response without calling the user's handler. Store `weak_ptr<ContextType>` to avoid reference cycles.

The service callback lambda captures a `weak_ptr` and locks it:

```cpp
template <typename ServiceT, typename ContextType> class Service {
  public:
    explicit Service(
        std::shared_ptr<ContextType> context,
        const std::string &service_name,
        const rclcpp::QoS &qos = rclcpp::ServicesQoS()
    ) {
        // ... default handler setup unchanged ...

        std::weak_ptr<ContextType> weak_ctx = context;
        service_ = context->node->template create_service<ServiceT>(
            service_name,
            [weak_ctx, this, service_name](
                const std::shared_ptr<typename ServiceT::Request> request,
                std::shared_ptr<typename ServiceT::Response> response
            ) {
                auto ctx = weak_ctx.lock();
                if (!ctx) return;
                if (ctx->node->get_current_state().id() !=
                    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
                    RCLCPP_WARN(
                        ctx->node->get_logger(),
                        "Service '%s': rejected, node not active",
                        service_name.c_str()
                    );
                    return;
                }
                request_handler_(ctx, request, response);
            },
            qos);
    }

    // set_request_handler signature unchanged
};
```

Requires `#include <lifecycle_msgs/msg/state.hpp>`.

Note: the old code captured `context` (a `shared_ptr`) in the service lambda, creating a cycle since the Service is owned by the context. Now it captures `weak_ctx` instead.

### `action_server.hpp`

Two changes:

1. Store `rclcpp_lifecycle::LifecycleNode *` instead of `rclcpp::Node *`:

```cpp
rclcpp_lifecycle::LifecycleNode *node_;
```

Update constructor signature, factory functions, and the context-based factory overload accordingly.

2. Reject goals when not Active, at the top of `handle_goal`:

```cpp
rclcpp_action::GoalResponse
handle_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const typename ActionT::Goal> goal) {
    if (node_->get_current_state().id() !=
        lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
        RCLCPP_WARN(
            node_->get_logger(),
            "Action server '%s': Rejecting goal, node not active",
            server_name_.c_str()
        );
        return rclcpp_action::GoalResponse::REJECT;
    }

    // ... existing validation logic unchanged ...
}
```

### `timer.hpp`

Both `create_timer` and `create_wall_timer` get three changes:

1. **Callback guard**: Wrap the user callback to only execute when Active.
2. **Start inactive when not Active**: Use `autostart=false` via the free-standing `rclcpp::create_timer` / `rclcpp::create_wall_timer` functions (the node member functions don't expose `autostart`, but the free-standing ones do).
3. **Capture `weak_ptr`**: Timer callbacks capture `weak_ptr<ContextType>` to avoid reference cycles (timers are stored in `ctx->timers`).

```cpp
template <typename DurationRepT, typename DurationT, typename ContextType, typename CallbackT>
auto create_timer(
    std::shared_ptr<ContextType> context,
    std::chrono::duration<DurationRepT, DurationT> period,
    CallbackT callback,
    rclcpp::CallbackGroup::SharedPtr group = nullptr
) {
    static_assert(std::is_base_of_v<Context, ContextType>);

    bool autostart = (context->node->get_current_state().id() ==
        lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

    std::weak_ptr<ContextType> weak_ctx = context;
    auto timer = rclcpp::create_timer(
        context->node,
        context->node->get_clock(),
        rclcpp::Duration(period),
        [weak_ctx, callback]() {
            auto ctx = weak_ctx.lock();
            if (!ctx) return;
            if (ctx->node->get_current_state().id() ==
                lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
                callback(ctx);
            }
        },
        group,
        autostart);

    context->timers.push_back(timer);
    return timer;
}

template <typename DurationRepT, typename DurationT, typename ContextType, typename CallbackT>
auto create_wall_timer(
    std::shared_ptr<ContextType> context,
    std::chrono::duration<DurationRepT, DurationT> period,
    CallbackT callback,
    rclcpp::CallbackGroup::SharedPtr group = nullptr
) {
    static_assert(std::is_base_of_v<Context, ContextType>);

    bool autostart = (context->node->get_current_state().id() ==
        lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

    std::weak_ptr<ContextType> weak_ctx = context;
    auto timer = rclcpp::create_wall_timer(
        period,
        [weak_ctx, callback]() {
            auto ctx = weak_ctx.lock();
            if (!ctx) return;
            if (ctx->node->get_current_state().id() ==
                lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
                callback(ctx);
            }
        },
        group,
        context->node->get_node_base_interface().get(),
        context->node->get_node_timers_interface().get(),
        autostart);

    context->timers.push_back(timer);
    return timer;
}
```

### `thread.hpp`

**Remove entirely.** Threads don't fit the lifecycle model — they hold long-lived references to the context, preventing cleanup, and require explicit stop/join coordination that cake can't manage generically. Users who need threads can manage them outside of cake. Timers and callback groups cover most use cases that `create_thread` was used for.

---

## Phase 3: Code generator

### 3a. Jinja2 template: `node_interface.hpp.jinja2`

This is the largest change. The generated `*Base` class changes fundamentally.

#### Template parameter signature

From:
```cpp
template <
    typename ContextType,
    auto init_func,
    auto extend_options = [](rclcpp::NodeOptions options) { return options; }>
```

To:
```cpp
template <
    typename ContextType,
    auto on_configure_func,
    auto on_activate_func = [](std::shared_ptr<ContextType>) {},
    auto on_deactivate_func = [](std::shared_ptr<ContextType>) {},
    auto on_cleanup_func = [](std::shared_ptr<ContextType>) {},
    auto extend_options = [](rclcpp::NodeOptions options) { return options; }>
```

#### Includes

Add:
```cpp
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/msg/state.hpp>
```

Remove (no longer needed in generated header, pulled in via cake headers):
```cpp
// rclcpp/rclcpp.hpp is still needed for NodeOptions in the template default
```

#### Constructor

The constructor does nothing beyond calling the base and the static_assert. `ctx_` starts as `nullptr` and is created fresh in each `on_configure`:

```cpp
explicit {{ class_name }}Base(const rclcpp::NodeOptions &options)
    : cake::BaseNode<"{{ node_name }}", extend_options>(options) {
    static_assert(
        std::is_base_of_v<{{ class_name }}Context<ContextType>, ContextType>,
        "ContextType must be a child of {{ class_name }}Context"
    );
}
```

#### New: `ctx_` member

```cpp
private:
    std::shared_ptr<ContextType> ctx_;
```

#### New: `on_configure` override

```cpp
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturn on_configure(const rclcpp_lifecycle::State &) override {
    ctx_ = std::make_shared<ContextType>();
    ctx_->node = this->shared_from_this();

    // init parameters
    ctx_->param_listener = std::make_shared<ParamListener>(ctx_->node);
    ctx_->params = ctx_->param_listener->get_params();

    // init publishers
    {%- for pub in publishers %}
    ctx_->publishers.{{ pub.field_name }} = cake::create_publisher<{{ pub.msg_type }}>(ctx_, {{ pub.name_expr }}, {{ pub.qos_code }});
    {%- endfor %}

    // init subscribers
    {%- for sub in subscribers %}
    ctx_->subscribers.{{ sub.field_name }} = cake::create_subscriber<{{ sub.msg_type }}>(ctx_, {{ sub.name_expr }}, {{ sub.qos_code }});
    {%- endfor %}

    // init services
    {%- for srv in services %}
    ctx_->services.{{ srv.field_name }} = cake::create_service<{{ srv.service_type }}>(ctx_, {{ srv.name_expr }});
    {%- endfor %}

    // init service clients
    {%- for client in service_clients %}
    ctx_->service_clients.{{ client.field_name }} = ctx_->node->template create_client<{{ client.service_type }}>({{ client.name_expr }});
    {%- endfor %}

    // init actions
    {%- for action in actions %}
    ctx_->actions.{{ action.field_name }} = cake::create_single_goal_action_server<{{ action.action_type }}>(ctx_, {{ action.name_expr }});
    {%- endfor %}

    // init action clients
    {%- for client in action_clients %}
    ctx_->action_clients.{{ client.field_name }} = rclcpp_action::create_client<{{ client.action_type }}>(ctx_->node, {{ client.name_expr }});
    {%- endfor %}

    on_configure_func(ctx_);
    return CallbackReturn::SUCCESS;
}
```

Note: `for_each_param` loops remain the same, just moved from the constructor into `on_configure`.

#### New: `on_activate` override

```cpp
CallbackReturn on_activate(const rclcpp_lifecycle::State &) override {
    // activate lifecycle publishers
    {%- for pub in publishers %}
    {%- if pub.for_each_param %}
    for (auto &[key, pub] : ctx_->publishers.{{ pub.field_name }}) { pub->activate(); }
    {%- else %}
    ctx_->publishers.{{ pub.field_name }}->activate();
    {%- endif %}
    {%- endfor %}

    // reset all timers
    for (auto &t : ctx_->timers) { t->reset(); }

    on_activate_func(ctx_);
    return CallbackReturn::SUCCESS;
}
```

#### New: `on_deactivate` override

```cpp
CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override {
    on_deactivate_func(ctx_);

    // cancel all timers
    for (auto &t : ctx_->timers) { t->cancel(); }

    // deactivate lifecycle publishers
    {%- for pub in publishers %}
    {%- if pub.for_each_param %}
    for (auto &[key, pub] : ctx_->publishers.{{ pub.field_name }}) { pub->deactivate(); }
    {%- else %}
    ctx_->publishers.{{ pub.field_name }}->deactivate();
    {%- endif %}
    {%- endfor %}

    return CallbackReturn::SUCCESS;
}
```

#### New: `on_cleanup` override

Destroy the context entirely. Because all wrappers and timer callbacks hold `weak_ptr<Context>` (not `shared_ptr`), there are no reference cycles. Dropping `ctx_` frees everything cleanly:

```cpp
CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override {
    on_cleanup_func(ctx_);
    ctx_.reset();
    return CallbackReturn::SUCCESS;
}
```

After `ctx_` is reset, any in-flight callbacks that try to `lock()` their `weak_ptr` will get `nullptr` and gracefully no-op. A fresh context is created on the next `on_configure`.

### 3b. Registration template: `node_registration.cpp.jinja2`

No changes. `RCLCPP_COMPONENTS_REGISTER_NODE` works with lifecycle nodes.

### 3c. Python generator: `generate_node_interface.py`

No schema changes to `interface.yaml`. The generator itself only changes in how it invokes the Jinja2 template — no new template variables are needed. All the new lifecycle code uses the same `publishers`, `subscribers`, `services`, `actions`, etc. lists that already exist.

### 3d. Python node template: `node_interface.py.jinja2`

Out of scope for this migration. The Python template uses `rclpy` and would need equivalent changes (`rclpy.lifecycle.LifecycleNode`), but can be done separately.

---

## Phase 4: Package configuration

### `cake/package.xml`

Add dependencies:

```xml
<depend>rclcpp_lifecycle</depend>
<depend>lifecycle_msgs</depend>
```

### `cake/CMakeLists.txt`

No changes needed — `ament_auto_find_build_dependencies()` picks up from `package.xml`.

### Downstream packages using cake

Every package that uses cake needs `rclcpp_lifecycle` and `lifecycle_msgs` in its `package.xml`. The `cake_auto_package()` CMake macro may need updating if it hardcodes dependency lists. Check whether it auto-pulls cake's dependencies or if downstream packages must list them explicitly.

---

## Phase 5: Update tests and examples

### Test fixtures

Every fixture in `cake/tests/fixtures/` has an `expected_cpp/` directory with golden-file outputs. All of these need regeneration to match the new template output.

Approach:
1. Update the Jinja2 template (Phase 3).
2. Run the generator against each fixture's `interface.yaml`.
3. Use `accept_outputs.sh` to update golden files.
4. Verify with `run_tests.sh`.

### `cake_example`

Update `cake_example/nodes/my_node/`:

**`my_node.hpp`** — rename `init` to `on_configure` in the typedef:
```cpp
using MyNode = MyNodeBase<Context, on_configure>;
```

**`my_node.cpp`** — rename `init` to `on_configure`:
```cpp
void on_configure(std::shared_ptr<Context> ctx) {
    // ... same body as current init() ...
}
```

Optionally add `on_activate` / `on_deactivate` to demonstrate the full lifecycle.

---

## Summary of file changes

| File | Change |
|---|---|
| `context.hpp` | `rclcpp::Node::SharedPtr` -> `rclcpp_lifecycle::LifecycleNode::SharedPtr` |
| `base_node.hpp` | Inherit from `LifecycleNode` instead of composing `rclcpp::Node` |
| `publisher.hpp` | Use `LifecyclePublisher`, add `activate()` / `deactivate()`, `weak_ptr` context |
| `subscriber.hpp` | Lifecycle state check in callback, `weak_ptr` context |
| `service.hpp` | Lifecycle state check, auto-reject when inactive, `weak_ptr` context |
| `action_server.hpp` | Change `rclcpp::Node*` to `LifecycleNode*`, reject goals when inactive |
| `timer.hpp` | Callback guard, `autostart` / `cancel()` based on state, `weak_ptr` context |
| `thread.hpp` | Remove entirely |
| `fixed_string.hpp` | No change |
| `qos_helpers.hpp` | No change |
| `node_interface.hpp.jinja2` | Major rewrite: constructor -> lifecycle callbacks |
| `node_registration.cpp.jinja2` | No change |
| `generate_node_interface.py` | No change (same template variables) |
| `package.xml` | Add `rclcpp_lifecycle`, `lifecycle_msgs` |
| `old/` | Delete entirely |
| All test fixtures | Regenerate golden files |
| `cake_example` | Rename `init` -> `on_configure` |

## Implementation order

Phases 1 and 2 can be done together (core headers + entity wrappers) since they're all header-only and form a coherent unit. Phase 3 (code generator) depends on phases 1-2 being done so the generated code matches the new APIs. Phase 4 (package config) can be done alongside anything. Phase 5 (tests/examples) must come after phase 3.

Suggested order: **4 -> 1+2 -> 3 -> 5**

Start with package.xml since it's trivial and unblocks compilation of everything else.

## Reconfigurability

The lifecycle state machine supports `Inactive -> on_cleanup -> Unconfigured -> on_configure -> Inactive` (reconfigure cycle). The design supports this cleanly because:

- All entity wrappers and callbacks hold `weak_ptr<Context>`, so there are no reference cycles.
- `on_cleanup` calls the user's `on_cleanup_func`, then does `ctx_.reset()`. That single call frees the entire context and all its entities. This works for timers because rclcpp's `CallbackGroup` stores `weak_ptr<TimerBase>` (not `shared_ptr`), so `ctx_->timers` holds the only owning references. When the context is destroyed, the timers are destroyed, and the callback group's weak pointers expire naturally — no explicit removal needed.
- Any in-flight callbacks (timers, subscribers) that fire after cleanup gracefully no-op — their `weak_ptr::lock()` returns `nullptr`.
- `on_configure` creates a fresh `ctx_` via `std::make_shared<ContextType>()` and repopulates everything from scratch.
- User state (e.g. `int counter = 0` on the derived context) is automatically reset to its default value on reconfigure — no user action needed.

This should be tested explicitly.
