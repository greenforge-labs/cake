# Cake Lifecycle Node Migration — Design Document

## Overview

Replace `rclcpp::Node` with `rclcpp_lifecycle::LifecycleNode` throughout the cake framework. After migration, all cake nodes are lifecycle nodes with the standard state machine: **Unconfigured → Inactive → Active → Finalized**.

## Architecture

### Three-layer design

The lifecycle implementation is split across three layers, each with a single responsibility:

1. **`BaseNode`** (hand-written, in cake library) — Lifecycle orchestration. Creates the lifecycle node, creates the context, registers lifecycle callbacks, and defines the state transition logic (when to create/activate/deactivate/destroy entities, when to call user callbacks, how to handle failure/error/shutdown). Provides virtual hooks for the layers below.

2. **Generated `*Base` class** (from Jinja2 template) — Entity plumbing. Overrides virtual methods to create/activate/deactivate the specific publishers, subscribers, services, timers, etc. defined in `interface.yaml`. Also wraps user callback template parameters into virtual method overrides that BaseNode calls.

3. **User class** (hand-written by the user) — Business logic. Provides the actual callback implementations as free functions passed via template parameters on the generated class.

### Composition

`BaseNode` wraps a `LifecycleNode::SharedPtr` member — same composition pattern as today. No inheritance from `LifecycleNode`.

`get_node_base_interface()` delegates to the wrapped node. `RCLCPP_COMPONENTS_REGISTER_NODE` continues to work.

### `BaseNode`

```cpp
template <fixed_string node_name, typename ContextType,
          auto extend_options = [](rclcpp::NodeOptions options) { return options; }>
class BaseNode {
  public:
    explicit BaseNode(const rclcpp::NodeOptions &options)
        : node_(std::make_shared<rclcpp_lifecycle::LifecycleNode>(node_name.c_str(), extend_options(options)))
    {
        auto ctx = std::make_shared<ContextType>();
        reset_context(ctx);

        // Lambdas capture ctx and route to methods
        node_->register_on_configure([this, ctx](const auto &) { return handle_configure(ctx); });
        node_->register_on_activate([this, ctx](const auto &) { return handle_activate(ctx); });
        node_->register_on_deactivate([this, ctx](const auto &) { return handle_deactivate(ctx); });
        node_->register_on_cleanup([this, ctx](const auto &) { return handle_cleanup(ctx); });
        node_->register_on_shutdown([this, ctx](const auto &) { return handle_shutdown(ctx); });
        node_->register_on_error([this, ctx](const auto &) { return handle_error(ctx); });
    }

    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() const {
        return node_->get_node_base_interface();
    }

  private:
    // Reset context to default-constructed state, preserving only node reference.
    // Resets ALL fields — entities AND user state.
    void reset_context(std::shared_ptr<ContextType> ctx) {
        *ctx = ContextType{};
        ctx->node = node_;
    }

    // Lifecycle orchestration — these define the transition logic
    CallbackReturn handle_configure(std::shared_ptr<ContextType> ctx) {
        create_entities(ctx);
        auto result = user_on_configure(ctx);
        if (result == CallbackReturn::FAILURE) { reset_context(ctx); }
        return result;
    }

    CallbackReturn handle_activate(std::shared_ptr<ContextType> ctx) {
        auto result = user_on_activate(ctx);
        if (result != CallbackReturn::SUCCESS) return result;
        activate_entities(ctx);
        return result;
    }

    CallbackReturn handle_deactivate(std::shared_ptr<ContextType> ctx) {
        auto result = user_on_deactivate(ctx);
        if (result != CallbackReturn::SUCCESS) return result;
        deactivate_entities(ctx);
        return result;
    }

    CallbackReturn handle_cleanup(std::shared_ptr<ContextType> ctx) {
        auto result = user_on_cleanup(ctx);
        if (result == CallbackReturn::SUCCESS) { reset_context(ctx); }
        return result;
    }

    CallbackReturn handle_shutdown(std::shared_ptr<ContextType> ctx) {
        user_on_shutdown(ctx);
        deactivate_entities(ctx);  // idempotent — safe even if never activated
        reset_context(ctx);
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn handle_error(std::shared_ptr<ContextType> ctx) {
        deactivate_entities(ctx);
        reset_context(ctx);
        return CallbackReturn::FAILURE;  // → Finalized
    }

  protected:
    // Entity lifecycle — generated class overrides these
    virtual void create_entities(std::shared_ptr<ContextType> ctx) {}
    virtual void activate_entities(std::shared_ptr<ContextType> ctx) {}
    virtual void deactivate_entities(std::shared_ptr<ContextType> ctx) {}

    // User callback hooks — generated class overrides to forward to template params
    virtual CallbackReturn user_on_configure(std::shared_ptr<ContextType> ctx) { return CallbackReturn::SUCCESS; }
    virtual CallbackReturn user_on_activate(std::shared_ptr<ContextType> ctx) { return CallbackReturn::SUCCESS; }
    virtual CallbackReturn user_on_deactivate(std::shared_ptr<ContextType> ctx) { return CallbackReturn::SUCCESS; }
    virtual CallbackReturn user_on_cleanup(std::shared_ptr<ContextType> ctx) { return CallbackReturn::SUCCESS; }
    virtual void user_on_shutdown(std::shared_ptr<ContextType> ctx) {}

    rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
};
```

Key points:
- `BaseNode` is templated on `node_name`, `ContextType`, and `extend_options`
- `ctx` is created as a local in the constructor and captured by the lambdas. It is not a member. The lambdas (and entity weak_ptrs) keep it alive.
- Lambdas are pure routing — capture `this` + `ctx`, forward to a `handle_*` method.
- `handle_*` methods encode the lifecycle orchestration: ordering of user callbacks vs entity management, failure/error handling.
- Virtual methods split into two groups: entity lifecycle (generated class fills in) and user callback hooks (generated class wraps template params).
- Virtual dispatch cost is irrelevant — lifecycle transitions are rare events.

### Generated class (sketch)

```cpp
template <
    typename ContextType,
    auto on_configure_func,
    auto on_activate_func = [](std::shared_ptr<ContextType>) { return CallbackReturn::SUCCESS; },
    auto on_deactivate_func = [](std::shared_ptr<ContextType>) { return CallbackReturn::SUCCESS; },
    auto on_cleanup_func = [](std::shared_ptr<ContextType>) { return CallbackReturn::SUCCESS; },
    auto on_shutdown_func = [](std::shared_ptr<ContextType>) {},
    auto extend_options = [](rclcpp::NodeOptions options) { return options; }>
class FooBase : public cake::BaseNode<"foo", ContextType, extend_options> {
  protected:
    // Entity lifecycle
    void create_entities(std::shared_ptr<ContextType> ctx) override {
        ctx->param_listener = std::make_shared<ParamListener>(ctx->node);
        ctx->params = ctx->param_listener->get_params();
        ctx->publishers.cmd_vel = cake::create_publisher<...>(ctx, "/cmd_vel", ...);
        ctx->subscribers.odom = cake::create_subscriber<...>(ctx, "/odom", ...);
        // ...
    }

    void activate_entities(std::shared_ptr<ContextType> ctx) override {
        ctx->publishers.cmd_vel->activate();
        // ...
        for (auto &t : ctx->timers) { t->reset(); }
    }

    void deactivate_entities(std::shared_ptr<ContextType> ctx) override {
        for (auto &t : ctx->timers) { t->cancel(); }
        ctx->publishers.cmd_vel->deactivate();
        // ...
    }

    // No destroy_entities — BaseNode::reset_context() handles full cleanup
    // via *ctx = ContextType{}, which resets all fields (entities + user state).

    // User callback forwarding
    CallbackReturn user_on_configure(std::shared_ptr<ContextType> ctx) override { return on_configure_func(ctx); }
    CallbackReturn user_on_activate(std::shared_ptr<ContextType> ctx) override { return on_activate_func(ctx); }
    CallbackReturn user_on_deactivate(std::shared_ptr<ContextType> ctx) override { return on_deactivate_func(ctx); }
    CallbackReturn user_on_cleanup(std::shared_ptr<ContextType> ctx) override { return on_cleanup_func(ctx); }
    void user_on_shutdown(std::shared_ptr<ContextType> ctx) override { on_shutdown_func(ctx); }
};
```

### Context ownership

The context is created once in the `BaseNode` constructor and lives for the lifetime of the node. Lifecycle transitions populate and depopulate its contents — the context shell itself is never destroyed or recreated.

The context mirrors the lifecycle state machine:

| Lifecycle state | Context state |
|---|---|
| **Unconfigured** | `ctx->node` is set. No params, no entities. (After construction, or after cleanup.) |
| **Inactive** | Fully populated — params, publishers, subscribers, services, actions, timers. Publishers not activated, timers not running. |
| **Active** | Everything is live. |

The context is never null. It is a long-lived shell that gets populated on configure and depopulated on cleanup.

#### Reconfigurability and `reset_context()`

The lifecycle state machine supports `Inactive → cleanup → Unconfigured → configure → Inactive` (reconfigure cycle). On cleanup (and on configure-failure, shutdown, and error), `reset_context()` resets the context via `*ctx = ContextType{}; ctx->node = node_`. This resets **all** fields — entities AND user state — back to their default-constructed values. Only `ctx->node` is re-assigned. On the next configure, `create_entities()` repopulates everything from scratch.

This means user state on a derived context (e.g. `int counter = 0`) is automatically reset on reconfigure — no user action needed. The `shared_ptr` itself doesn't change (all lambdas still point to the same object), only its contents are reset.

### `context.hpp`

```cpp
struct Context {
    rclcpp_lifecycle::LifecycleNode::SharedPtr node;
    std::vector<rclcpp::TimerBase::SharedPtr> timers;
};
```

`threads` vector is removed — threads don't fit the lifecycle model (see Thread Removal below).

### `weak_ptr` in entity wrappers

Entity wrappers (Publisher, Subscriber, Service) and timer callbacks store `weak_ptr<Context>` instead of `shared_ptr<Context>`. This eliminates reference cycles between the context and its entities. On cleanup, resetting entity fields on the context is sufficient — no custom destructor logic needed. In-flight callbacks that try to `lock()` after cleanup get `nullptr` and gracefully no-op.

---

## User callback API

Cake owns the ROS entity lifecycle — creating, activating, deactivating, and destroying publishers, subscribers, services, timers, and action servers. The user callbacks are for managing custom state and external resources that cake doesn't know about.

### Callbacks

Five user callbacks, passed as template parameters on the generated class:

| Callback | Required? | Return | Purpose |
|---|---|---|---|
| `on_configure` | **Yes** (no default) | `CallbackReturn` | Replaces `init`. Wire up subscriber/service callbacks, create timers, initialize custom context fields. |
| `on_activate` | No | `CallbackReturn` | Connect external systems, arm hardware. Most users skip. |
| `on_deactivate` | No | `CallbackReturn` | Disconnect external systems, disarm hardware. Most users skip. |
| `on_cleanup` | No | `CallbackReturn` | Release non-RAII resources. Most users skip. |
| `on_shutdown` | No | `void` | Final cleanup before teardown. Cannot block shutdown. |

`on_configure` is required because it doesn't make sense to set up entities without providing handlers for them.

If all custom state lives on the context and uses RAII, only `on_configure` needs a user implementation.

### Ordering relative to entity management

The ordering of user callbacks vs cake's entity management is encoded in `BaseNode::handle_*` methods:

- **configure**: `create_entities` → `user_on_configure`. Entities exist when the user callback runs, so the user can wire up subscriber/service callbacks and create timers.
- **activate**: `user_on_activate` → `activate_entities`. User runs first. If the user returns FAILURE, nothing was activated — no rollback needed.
- **deactivate**: `user_on_deactivate` → `deactivate_entities`. User runs first. Publishers are still activated during the user callback — the user can publish a final status.
- **cleanup**: `user_on_cleanup` → `reset_context`. User runs first, then the context is wiped.
- **shutdown**: `user_on_shutdown` → `deactivate_entities` → `reset_context`. Always called, always succeeds.

### `on_shutdown` details

`on_shutdown` has `void` return — it cannot block shutdown. The lifecycle state machine does not allow failure on the shutdown transition, and throwing an exception is handled externally to cake (by rclcpp).

`on_shutdown` is always called regardless of the node's current primary state (Active, Inactive, or Unconfigured). From Unconfigured the context exists but is in its default-constructed state (only `ctx->node` is set). This is simpler than guarding — principle of least surprise.

---

## Entity behavior during lifecycle transitions

There is an intentional asymmetry between publishers (outbound) and everything else (inbound/scheduled).

**Rationale**: Publishers are outbound — the node decides when to publish, and sending a final message during teardown is valuable. Everything else involves accepting inbound work or executing scheduled logic; letting that fire during teardown risks running user code in a partially-torn-down state.

All entity wrappers store `weak_ptr<ContextType>` instead of `shared_ptr<ContextType>` to eliminate reference cycles. Callbacks lock before use; if the context is gone, the callback is a no-op.

### `publisher.hpp`

Use `LifecyclePublisher` under the hood. Publishing is a silent no-op when deactivated. Controlled by explicit `activate()` / `deactivate()` calls in `activate_entities` / `deactivate_entities` — not by node state checks. This means they stay functional during transitions until `deactivate_entities()` actually runs. The user can publish from `on_deactivate` and `on_shutdown` callbacks.

```cpp
template <typename MessageT, typename ContextType> class Publisher {
  public:
    explicit Publisher(std::shared_ptr<ContextType> context, const std::string &topic_name, const rclcpp::QoS &qos)
        : context_(context) {
        rclcpp::PublisherOptions options;
        options.event_callbacks.deadline_callback = [this](rclcpp::QOSDeadlineOfferedInfo &event) {
            if (auto ctx = context_.lock()) {
                if (deadline_callback_) { deadline_callback_(ctx, event); }
            }
        };
        options.event_callbacks.liveliness_callback = [this](rclcpp::QOSLivelinessLostInfo &event) {
            if (auto ctx = context_.lock()) {
                if (liveliness_callback_) { liveliness_callback_(ctx, event); }
            }
        };

        publisher_ = context->node->template create_publisher<MessageT>(topic_name, qos, options);
    }

    void publish(const MessageT &msg) { publisher_->publish(msg); }
    void publish(std::unique_ptr<MessageT> msg) { publisher_->publish(std::move(msg)); }

    void activate() { publisher_->on_activate(); }
    void deactivate() { publisher_->on_deactivate(); }

    typename rclcpp_lifecycle::LifecyclePublisher<MessageT>::SharedPtr publisher() { return publisher_; }

    // set_deadline_callback, set_liveliness_callback unchanged

  private:
    std::weak_ptr<ContextType> context_;
    typename rclcpp_lifecycle::LifecyclePublisher<MessageT>::SharedPtr publisher_;
    // ...
};
```

Notes:
- `LifecycleNode::create_publisher` returns `LifecyclePublisher`, which silently drops `publish()` calls when deactivated.
- `activate()` / `deactivate()` are called by generated `activate_entities` / `deactivate_entities` — not by user code.
- The `publisher()` accessor return type changes from `rclcpp::Publisher<MessageT>::SharedPtr` to `rclcpp_lifecycle::LifecyclePublisher<MessageT>::SharedPtr`. Breaking change for anyone calling `publisher()` directly.

### `subscriber.hpp`

Silently drop messages when not Active. `PRIMARY_STATE_ACTIVE` check in the subscription callback lambda:

```cpp
template <typename MessageT, typename ContextType> class Subscriber {
  public:
    explicit Subscriber(std::shared_ptr<ContextType> context, const std::string &topic_name, const rclcpp::QoS &qos)
        : context_(context) {
        // ... default callback setup, QoS event callbacks unchanged ...

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

  private:
    std::weak_ptr<ContextType> context_;
    // ...
};
```

### `service.hpp`

Log a warning and return a default-constructed response when not Active. (ROS2 services have no reject mechanism — the callback always runs and always produces a response. A default-constructed response + warning log is the best cake can do generically.)

Also fixes an existing reference cycle — the old code captured `context` (a `shared_ptr`) in the service lambda. Now captures `weak_ptr`.

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
};
```

### `action_server.hpp`

Reject goals when not Active. Unlike services, the action protocol has a real reject mechanism.

Two changes:

1. Store `rclcpp_lifecycle::LifecycleNode *` instead of `rclcpp::Node *`. Update constructor, factory functions, and context-based factory overload accordingly.

2. Reject goals at the top of `handle_goal`:

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

Three changes to both `create_timer` and `create_wall_timer`:

1. **Callback guard**: Wrap user callback to only execute when Active (belt-and-suspenders — catches edge cases like race conditions).
2. **Start inactive when not Active**: Use `autostart=false` via freestanding `rclcpp::create_timer` / `rclcpp::create_wall_timer` (the node member functions don't expose `autostart`, but the freestanding ones do).
3. **Capture `weak_ptr`**: Timer callbacks capture `weak_ptr<ContextType>` to avoid reference cycles (timers are stored in `ctx->timers`).

`activate_entities` calls `reset()` on all timers. `deactivate_entities` calls `cancel()`.

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
```

`create_wall_timer` follows the same pattern using `rclcpp::create_wall_timer` with `get_node_base_interface()` and `get_node_timers_interface()`.

---

## Error handling and `CallbackReturn` semantics

All user lifecycle callbacks (except `on_shutdown`) return `CallbackReturn`. The three values map to the rclcpp lifecycle state machine:

- **`SUCCESS`** — Transition proceeds normally to the target state.
- **`FAILURE`** — Transition is rolled back to the previous primary state. The node can retry. Use this for transient problems: a config file isn't available yet, a hardware connection timed out, an external dependency isn't ready.
- **`ERROR`** — Unrecoverable error. The node enters ErrorProcessing, `on_error` fires (cleans up), and the node goes to **Finalized** (terminal). The node cannot be reconfigured — it must be destroyed and recreated. Use this for corrupted internal state, hardware faults, invariant violations.

What cake does on each return value (encoded in `BaseNode::handle_*` methods):

| Transition | SUCCESS | FAILURE | ERROR |
|---|---|---|---|
| configure | → Inactive | `reset_context()` → Unconfigured | `handle_error` → Finalized |
| activate | `activate_entities()` → Active | nothing to roll back → Inactive | `handle_error` → Finalized |
| deactivate | `deactivate_entities()` → Inactive | stays Active (nothing torn down) | `handle_error` → Finalized |
| cleanup | `reset_context()` → Unconfigured | stays Inactive (context preserved) | `handle_error` → Finalized |

`handle_error` does `deactivate_entities(ctx); reset_context(ctx); return FAILURE;` — idempotent cleanup from any state, then Finalized. There is no user callback for `on_error` — ERROR is unrecoverable, there's nothing useful the user can do.

### Exception handling

No try/catch in cake. rclcpp's `execute_callback` wraps every transition callback in `try/catch(const std::exception&)`. If a user callback throws:

1. rclcpp catches it and logs the error.
2. The return code is set to `CallbackReturn::ERROR`.
3. The state machine enters ErrorProcessing → `handle_error` fires → Finalized.

This is sufficient because `handle_error` can tear down from any state — `deactivate_entities` is idempotent and `reset_context` wipes everything. No intermediate catch needed.

Only `std::exception` subclasses are caught. A non-`std::exception` throw (e.g. `throw 42`) propagates uncaught and terminates the process.

---

## Thread removal

`thread.hpp` is removed entirely. Threads don't fit the lifecycle model — they hold long-lived references to the context, preventing cleanup, and require explicit stop/join coordination that cake can't manage generically. Users who need threads can manage them outside of cake. Timers and callback groups cover most use cases that `create_thread` was used for.

The `threads` vector is also removed from `context.hpp`.

---

## Package configuration

Add dependencies to `cake/package.xml`:

```xml
<depend>rclcpp_lifecycle</depend>
<depend>lifecycle_msgs</depend>
```

`cake/CMakeLists.txt` uses `ament_auto_find_build_dependencies()`, so it picks these up from `package.xml` automatically — no CMake changes needed.

Downstream packages that use cake also need `rclcpp_lifecycle` and `lifecycle_msgs` in their `package.xml`. `cake_auto_package()` uses `ament_auto_find_build_dependencies()` which reads the downstream package's own `package.xml`, so adding the deps there is sufficient.

Note: `cake_auto_package.cmake` hardcodes `target_link_libraries(... INTERFACE rclcpp::rclcpp)` for generated interface libraries (line 255). This may need `rclcpp_lifecycle::rclcpp_lifecycle` added, or it may work transitively through cake's headers — verify during implementation.

---

## Code generator changes

### Jinja2 template: `node_interface.hpp.jinja2`

The generated `*Base` class changes from a monolithic constructor to virtual method overrides on `BaseNode`.

**Template parameter signature** changes from:

```cpp
template <
    typename ContextType,
    auto init_func,
    auto extend_options = [](rclcpp::NodeOptions options) { return options; }>
```

To:

```cpp
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

template <
    typename ContextType,
    auto on_configure_func,
    auto on_activate_func = [](std::shared_ptr<ContextType>) { return CallbackReturn::SUCCESS; },
    auto on_deactivate_func = [](std::shared_ptr<ContextType>) { return CallbackReturn::SUCCESS; },
    auto on_cleanup_func = [](std::shared_ptr<ContextType>) { return CallbackReturn::SUCCESS; },
    auto on_shutdown_func = [](std::shared_ptr<ContextType>) {},
    auto extend_options = [](rclcpp::NodeOptions options) { return options; }>
```

The `using CallbackReturn` alias is emitted at namespace scope before the class definition, so it's available in the template parameter defaults.

**Base class** changes from `cake::BaseNode<"{{ node_name }}", extend_options>` to `cake::BaseNode<"{{ node_name }}", ContextType, extend_options>`.

**Constructor replaced by virtual overrides.** The current constructor creates ctx, creates all entities, and calls `init_func`. In the new design, the constructor just calls the BaseNode constructor. The generated class overrides:

- `create_entities(ctx)` — params, publishers, subscribers, services, service clients, actions, action clients. Same code as the current constructor body, just moved into this method. `for_each_param` loops are identical.
- `activate_entities(ctx)` — activate all publishers, reset all timers.
- `deactivate_entities(ctx)` — cancel all timers, deactivate all publishers.
- `user_on_configure(ctx)` → `on_configure_func(ctx)`
- `user_on_activate(ctx)` → `on_activate_func(ctx)`
- `user_on_deactivate(ctx)` → `on_deactivate_func(ctx)`
- `user_on_cleanup(ctx)` → `on_cleanup_func(ctx)`
- `user_on_shutdown(ctx)` → `on_shutdown_func(ctx)`

**Includes** — add `rclcpp_lifecycle/lifecycle_node.hpp`. `lifecycle_msgs/msg/state.hpp` is not needed in generated code (it's in the entity wrappers).

**`name_expr` compatibility** — The Python generator hardcodes `ctx->params.X` in `name_expr` values. This continues to work because the parameter to `create_entities` is named `ctx`.

**Generated class sketch** (same as in Architecture section above, but for reference):

```cpp
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

template <
    typename ContextType,
    auto on_configure_func,
    auto on_activate_func = [](std::shared_ptr<ContextType>) { return CallbackReturn::SUCCESS; },
    auto on_deactivate_func = [](std::shared_ptr<ContextType>) { return CallbackReturn::SUCCESS; },
    auto on_cleanup_func = [](std::shared_ptr<ContextType>) { return CallbackReturn::SUCCESS; },
    auto on_shutdown_func = [](std::shared_ptr<ContextType>) {},
    auto extend_options = [](rclcpp::NodeOptions options) { return options; }>
class {{ class_name }}Base : public cake::BaseNode<"{{ node_name }}", ContextType, extend_options> {
    static_assert(
        std::is_base_of_v<{{ class_name }}Context<ContextType>, ContextType>,
        "ContextType must derive from {{ class_name }}Context"
    );

  protected:
    void create_entities(std::shared_ptr<ContextType> ctx) override {
        ctx->param_listener = std::make_shared<ParamListener>(ctx->node);
        ctx->params = ctx->param_listener->get_params();

        // publishers, subscribers, services, service_clients, actions, action_clients
        // (same Jinja2 loops as current template, referencing ctx->)
    }

    void activate_entities(std::shared_ptr<ContextType> ctx) override {
        {%- for pub in publishers %}
        {%- if pub.for_each_param %}
        for (auto &[key, pub] : ctx->publishers.{{ pub.field_name }}) { pub->activate(); }
        {%- else %}
        ctx->publishers.{{ pub.field_name }}->activate();
        {%- endif %}
        {%- endfor %}
        for (auto &t : ctx->timers) { t->reset(); }
    }

    void deactivate_entities(std::shared_ptr<ContextType> ctx) override {
        for (auto &t : ctx->timers) { t->cancel(); }
        {%- for pub in publishers %}
        {%- if pub.for_each_param %}
        for (auto &[key, pub] : ctx->publishers.{{ pub.field_name }}) { pub->deactivate(); }
        {%- else %}
        ctx->publishers.{{ pub.field_name }}->deactivate();
        {%- endif %}
        {%- endfor %}
    }

    CallbackReturn user_on_configure(std::shared_ptr<ContextType> ctx) override { return on_configure_func(ctx); }
    CallbackReturn user_on_activate(std::shared_ptr<ContextType> ctx) override { return on_activate_func(ctx); }
    CallbackReturn user_on_deactivate(std::shared_ptr<ContextType> ctx) override { return on_deactivate_func(ctx); }
    CallbackReturn user_on_cleanup(std::shared_ptr<ContextType> ctx) override { return on_cleanup_func(ctx); }
    void user_on_shutdown(std::shared_ptr<ContextType> ctx) override { on_shutdown_func(ctx); }
};
```

**Entity struct definitions** (Publishers, Subscribers, Services, etc.) and the **Context struct definition** are unchanged.

### `static_assert` placement

Two levels of assertion:

1. **`BaseNode`** asserts `ContextType` derives from `cake::Context`:
   ```cpp
   static_assert(std::is_base_of_v<Context, ContextType>,
       "ContextType must derive from cake::Context");
   ```

2. **Generated class** asserts `ContextType` derives from the specific generated context:
   ```cpp
   static_assert(std::is_base_of_v<{{ class_name }}Context<ContextType>, ContextType>,
       "ContextType must derive from {{ class_name }}Context");
   ```

3. **Entity wrappers** (`publisher.hpp`, `subscriber.hpp`, `service.hpp`, `action_server.hpp`, `timer.hpp`) all assert `ContextType` derives from `cake::Context`:
   ```cpp
   static_assert(std::is_base_of_v<Context, ContextType>,
       "ContextType must derive from cake::Context");
   ```
   Currently only `timer.hpp` has this. Add to `publisher.hpp`, `subscriber.hpp`, `service.hpp`, and `action_server.hpp`.

### Registration template: `node_registration.cpp.jinja2`

No changes. `RCLCPP_COMPONENTS_REGISTER_NODE` works with lifecycle nodes.

### Python generator: `generate_node_interface.py`

No schema changes to `interface.yaml`. No new template variables needed. The generator itself only changes in how it renders the Jinja2 template — same `publishers`, `subscribers`, `services`, `actions`, etc. lists.

### Python node template: `node_interface.py.jinja2`

Out of scope for this migration.

---

## Tests and examples

Every fixture in `cake/tests/fixtures/` has an `expected_cpp/` directory with golden-file outputs. All need regeneration to match the new template output.

Workflow:
1. Update the Jinja2 template and entity wrappers.
2. Run the generator against each fixture's `input.yaml`.
3. Use `accept_outputs.sh` to update golden files.
4. Verify with `run_tests.sh`.

`cake_example` needs updating: rename `init` → `on_configure`, return `CallbackReturn::SUCCESS`.
