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

## Decisions (to be discussed)

The following sections are placeholders for decisions that still need to be worked through and documented:

### Entity behavior during lifecycle transitions

There is an intentional asymmetry between publishers (outbound) and everything else (inbound/scheduled).

**Publishers** use `LifecyclePublisher` under the hood. Publishing is a silent no-op when deactivated. They are controlled by explicit `activate()` / `deactivate()` calls in `activate_entities` / `deactivate_entities` — not by node state checks. This means they stay functional during transitions until `deactivate_entities()` actually runs. The user can publish from `on_deactivate` and `on_shutdown` callbacks (useful for sending a final status or safe command to hardware).

**Subscribers, Services, Action Servers, and Timers** are all guarded by a `PRIMARY_STATE_ACTIVE` check in their callback wrappers, inside the entity wrapper classes themselves (`subscriber.hpp`, `service.hpp`, etc.). This means the guard is always on — no generated code needed.

- **Subscribers**: Silently drop messages when not Active.
- **Services**: Log a warning and return a default-constructed response. (ROS2 services have no reject mechanism — the callback always runs and always produces a response. A default-constructed response + warning log is the best cake can do generically.)
- **Action servers**: Reject goals with `GoalResponse::REJECT` and log a warning. (Unlike services, the action protocol has a real reject mechanism.)
- **Timers**: Callback guard (no-op when not Active) + `cancel()` / `reset()` in `deactivate_entities` / `activate_entities`. Also `autostart=false` when created outside Active state. The callback guard is belt-and-suspenders — `cancel()` should prevent firing, but the guard catches edge cases like race conditions.

**Rationale**: Publishers are outbound — the node decides when to publish, and sending a final message during teardown is valuable. Everything else involves accepting inbound work or executing scheduled logic; letting that fire during teardown risks running user code in a partially-torn-down state.

### Error handling and `CallbackReturn` semantics

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

#### Exception handling

No try/catch in cake. rclcpp's `execute_callback` wraps every transition callback in `try/catch(const std::exception&)`. If a user callback throws:

1. rclcpp catches it and logs the error.
2. The return code is set to `CallbackReturn::ERROR`.
3. The state machine enters ErrorProcessing → `handle_error` fires → Finalized.

This is sufficient because `handle_error` can tear down from any state — `deactivate_entities` is idempotent and `reset_context` wipes everything. No intermediate catch needed.

Only `std::exception` subclasses are caught. A non-`std::exception` throw (e.g. `throw 42`) propagates uncaught and terminates the process.

### Shutdown

`on_shutdown` behavior from each primary state. Void return (cannot block). Relationship to `on_deactivate` / `on_cleanup`.

### Code generator changes

Jinja2 template rewrite. Python generator changes. Template parameter signature.

### Thread removal

`thread.hpp` removed. Rationale and alternatives.

### Package configuration

`rclcpp_lifecycle` and `lifecycle_msgs` dependencies. Downstream package impact.

### Tests and examples

Golden file regeneration. `cake_example` migration.
