# Cake Lifecycle Node Migration — Design Document

## Overview

Replace `rclcpp::Node` with `rclcpp_lifecycle::LifecycleNode` throughout the cake framework. After migration, all cake nodes are lifecycle nodes with the standard state machine: **Unconfigured → Inactive → Active → Finalized**.

## Architecture

### Three-layer design

The lifecycle implementation is split across three layers, each with a single responsibility:

1. **`BaseNode`** (hand-written, in cake library) — Lifecycle orchestration. Creates the lifecycle node, creates the context, registers lifecycle callbacks, and defines the state transition logic (when to create/activate/deactivate/destroy entities, when to call user callbacks, how to handle failure/error/shutdown). Provides virtual hooks for the layers below.

2. **Generated `*Base` class** (from Jinja2 template) — Entity plumbing. Overrides virtual methods to create/activate/deactivate/destroy the specific publishers, subscribers, services, timers, etc. defined in `interface.yaml`. Also wraps user callback template parameters into virtual method overrides that BaseNode calls.

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
        ctx->node = node_;

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
    // Lifecycle orchestration — these define the transition logic
    CallbackReturn handle_configure(std::shared_ptr<ContextType> ctx) {
        create_entities(ctx);
        auto result = user_on_configure(ctx);
        if (result == CallbackReturn::FAILURE) { destroy_entities(ctx); }
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
        if (result == CallbackReturn::SUCCESS) { destroy_entities(ctx); }
        return result;
    }

    CallbackReturn handle_shutdown(std::shared_ptr<ContextType> ctx) {
        user_on_shutdown(ctx);
        deactivate_entities(ctx);  // idempotent — safe even if never activated
        destroy_entities(ctx);
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn handle_error(std::shared_ptr<ContextType> ctx) {
        deactivate_entities(ctx);
        destroy_entities(ctx);
        return CallbackReturn::FAILURE;  // → Finalized
    }

  protected:
    // Entity lifecycle — generated class overrides these
    virtual void create_entities(std::shared_ptr<ContextType> ctx) {}
    virtual void activate_entities(std::shared_ptr<ContextType> ctx) {}
    virtual void deactivate_entities(std::shared_ptr<ContextType> ctx) {}
    virtual void destroy_entities(std::shared_ptr<ContextType> ctx) {}

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
- `BaseNode` is templated on `node_name`, `ContextType`, and `extend_options` only — no user callback template parameters.
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

    void destroy_entities(std::shared_ptr<ContextType> ctx) override {
        ctx->publishers.cmd_vel.reset();
        ctx->subscribers.odom.reset();
        ctx->param_listener.reset();
        ctx->timers.clear();
        // ...
    }

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

#### Reconfigurability

The lifecycle state machine supports `Inactive → cleanup → Unconfigured → configure → Inactive` (reconfigure cycle). On cleanup, `destroy_entities()` resets entity fields on the context (publishers, subscribers, etc. set to null). On the next configure, `create_entities()` recreates them from scratch. The context shell persists — user state on a derived context is **not** automatically reset. If users need state reset on reconfigure, they do it explicitly in `on_configure` or `on_cleanup`.

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

## Decisions (to be discussed)

The following sections are placeholders for decisions that still need to be worked through and documented:

### User callback API

`on_configure` required (replaces `init`), others optional. Return types and `CallbackReturn` semantics. Ordering of user callbacks relative to entity management (user runs first on activate, user runs first on deactivate, etc.).

### Entity behavior during lifecycle transitions

How each entity type (publishers, subscribers, services, action servers, timers) behaves in each lifecycle state and during transitions. The intentional asymmetry between publishers (outbound, controlled by explicit activate/deactivate) and everything else (inbound, guarded by state check).

### Error handling and `CallbackReturn` semantics

`SUCCESS` / `FAILURE` / `ERROR` meanings. Exception handling (delegated to rclcpp). `on_error` behavior.

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
