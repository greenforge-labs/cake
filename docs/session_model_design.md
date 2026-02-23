# Cake Session Model Redesign

## Problem

The current cake framework uses a `Context` struct that bundles a `shared_ptr<LifecycleNode>` with per-cycle state (timers, publishers, subscribers, params). The context is created once in the `BaseNode` constructor and reset in-place (`*ctx = ContextType{}; ctx->node = node_`) on cleanup/shutdown/error transitions.

This has two problems:

1. **No "no context" state**: During in-place reset there is a gap where `ctx->node` is null but the shared_ptr is still alive, so `weak_ptr::lock()` in entity callbacks succeeds and hands back a context with a null node. The single-threaded executor assumption papers over this.

2. **Lifecycle semantics are implicit**: Nothing in the type system communicates that context is born at configure and dies at cleanup. It looks like any other long-lived shared_ptr.

## Design Decisions

### 1. Rename `Context` to `Session`

"Session" communicates a transient, lifecycle-scoped resource. A session is **created** at configure and **destroyed** at cleanup/shutdown -- there is no in-place reset.

### 2. Node reference lives inside Session as a C++ reference

```cpp
rclcpp_lifecycle::LifecycleNode& node;
```

- Set once at construction, can never be null
- Since sessions are created fresh (not reset in-place), the non-assignable constraint of a reference member is irrelevant
- User code: `session->node.get_logger()` (dot, not arrow)

We considered pulling `node` out of Session entirely and plumbing it as a separate parameter to all callbacks. This worked but added verbosity. The key insight was that once we stopped doing in-place reset, the original motivation for pulling node out (protecting it from being nulled during reset) disappeared. A reference gives us a compile-time guarantee that node is always valid for the session's lifetime.

### 3. Session lifetime: create and destroy, never reset

- `handle_configure()`: creates a new `shared_ptr<SessionType>` via `create_session()`, then calls `user_on_configure()`
- `handle_cleanup()`: calls `user_on_cleanup()`, then `session_.reset()` -- session dies, all entity weak_ptrs go dead
- `handle_shutdown()`: calls `user_on_shutdown()`, deactivates entities, then `session_.reset()`
- `handle_error()`: deactivates entities, then `session_.reset()`
- No `reset_context()` function at all

### 4. Generated code creates the session, user code receives it

- `create_session(LifecycleNode&)` is the generated layer's virtual -- it builds the session, populates params, creates all entities
- `user_on_configure(std::shared_ptr<SessionType>)` is the user's virtual -- receives a fully populated session
- The user never constructs or destroys sessions

This makes the lifecycle relationship structural: configure *produces* a session, the user *receives* it ready-made. The session's existence implies the node has been configured.

### 5. Entity lifetime tied to session via rclcpp ownership model

rclcpp's internal ownership model validates this approach:
- Callback groups and executors store entities (subscriptions, timers, services) as **weak_ptrs**
- The `shared_ptr` returned from `create_subscription()`, `create_timer()`, etc. is the **sole owner**
- Session holds these shared_ptrs as members
- When `session_.reset()` destroys the session, all entity shared_ptrs drop, entities are destroyed, executor/callback-group weak_ptrs go dead, and callbacks stop firing
- No explicit cleanup code needed -- dropping the session unwinds everything

### 6. Weak pointer lock keeps session alive during callbacks

Entity callbacks (subscriber message handler, timer tick, service request) hold `weak_ptr<SessionType>`. When a callback fires:

1. Executor locks its weak_ptr to the entity -- entity stays alive for callback duration
2. Callback locks `weak_ptr<SessionType>` -- if session is alive, proceed with a local `shared_ptr`; if not, bail
3. The local `shared_ptr` keeps the session alive until the callback returns, even if `session_.reset()` runs concurrently

In a single-threaded executor, lifecycle transitions and entity callbacks are serialized on the same thread, so concurrent destruction can't happen. But the weak_ptr pattern is correct for multi-threaded executors too.

### 7. `this` capture in lifecycle lambdas

Lifecycle lambdas capture `this` (raw `BaseNode*`) to reach `handle_configure()`, `create_session()`, `user_on_configure()`, etc.

- This is unavoidable with composition + virtual methods -- the lambdas need vtable dispatch
- We chose composition deliberately to separate node dynamics from user code; the session is the bridge between them
- Inheritance from `LifecycleNode` would eliminate the capture problem but would expose lifecycle internals to user code
- BaseNode must outlive the LifecycleNode, which is guaranteed by construction (BaseNode owns `node_`)

## Ownership Model

```
BaseNode (root of execution, first thing constructed)
  |-- node_ (shared_ptr<LifecycleNode>) -- permanent, owns the node
  |     '-- lifecycle lambdas capture `this` (raw ptr to BaseNode)
  '-- session_ (shared_ptr<SessionType>, nullable) -- per-cycle
        |-- node (reference to *node_) -- always valid while session exists
        |-- timers (vector<shared_ptr>) -- owned by session
        |-- publishers.X (shared_ptr<Publisher>) -- owned by session
        |-- subscribers.X (shared_ptr<Subscriber>) -- owned by session
        |-- services.X (shared_ptr<Service>) -- owned by session
        |-- actions.X (shared_ptr<ActionServer>) -- owned by session
        |-- param_listener (shared_ptr<ParamListener>)
        '-- params (Params struct)

Entity callbacks (subscriber, timer, service):
  '-- weak_ptr<SessionType> -- lock() to get session for callback duration

rclcpp internals (callback groups, executor):
  '-- weak_ptr to entities -- go dead when session drops
```

## Proposed Code

### `cake::Session` (replaces `cake::Context`)

```cpp
struct Session {
    rclcpp_lifecycle::LifecycleNode& node;
    std::vector<rclcpp::TimerBase::SharedPtr> timers;
    explicit Session(rclcpp_lifecycle::LifecycleNode& n) : node(n) {}
};
```

### `cake::BaseNode` (updated)

```cpp
template <fixed_string node_name, typename SessionType,
    auto extend_options = [](rclcpp::NodeOptions options) { return options; }>
class BaseNode {
    static_assert(std::is_base_of_v<Session, SessionType>,
        "SessionType must derive from cake::Session");

  public:
    explicit BaseNode(const rclcpp::NodeOptions &options)
        : node_(std::make_shared<rclcpp_lifecycle::LifecycleNode>(
              node_name.c_str(), extend_options(options))) {
        node_->register_on_configure([this](const auto &) { return handle_configure(); });
        node_->register_on_activate([this](const auto &) { return handle_activate(); });
        node_->register_on_deactivate([this](const auto &) { return handle_deactivate(); });
        node_->register_on_cleanup([this](const auto &) { return handle_cleanup(); });
        node_->register_on_shutdown([this](const auto &) { return handle_shutdown(); });
        node_->register_on_error([this](const auto &) { return handle_error(); });
    }

    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() const {
        return node_->get_node_base_interface();
    }

  private:
    CallbackReturn handle_configure() {
        session_ = create_session(*node_);
        if (!session_) return CallbackReturn::FAILURE;
        auto result = user_on_configure(session_);
        if (result == CallbackReturn::FAILURE) {
            session_.reset();
        }
        return result;
    }

    CallbackReturn handle_activate() {
        auto result = user_on_activate(session_);
        if (result != CallbackReturn::SUCCESS) return result;
        activate_entities(session_);
        return result;
    }

    CallbackReturn handle_deactivate() {
        auto result = user_on_deactivate(session_);
        if (result != CallbackReturn::SUCCESS) return result;
        deactivate_entities(session_);
        return result;
    }

    CallbackReturn handle_cleanup() {
        auto result = user_on_cleanup(session_);
        session_.reset();
        return result;
    }

    CallbackReturn handle_shutdown() {
        user_on_shutdown(session_);
        deactivate_entities(session_);
        session_.reset();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn handle_error() {
        deactivate_entities(session_);
        session_.reset();
        return CallbackReturn::FAILURE;
    }

  protected:
    // Generated layer -- overridden by code-generated base class
    virtual std::shared_ptr<SessionType> create_session(rclcpp_lifecycle::LifecycleNode &) {
        return std::make_shared<SessionType>();
    }
    virtual void activate_entities(std::shared_ptr<SessionType>) {}
    virtual void deactivate_entities(std::shared_ptr<SessionType>) {}

    // User layer -- overridden by user's node class
    virtual CallbackReturn user_on_configure(std::shared_ptr<SessionType>) { return CallbackReturn::SUCCESS; }
    virtual CallbackReturn user_on_activate(std::shared_ptr<SessionType>) { return CallbackReturn::SUCCESS; }
    virtual CallbackReturn user_on_deactivate(std::shared_ptr<SessionType>) { return CallbackReturn::SUCCESS; }
    virtual CallbackReturn user_on_cleanup(std::shared_ptr<SessionType>) { return CallbackReturn::SUCCESS; }
    virtual void user_on_shutdown(std::shared_ptr<SessionType>) {}

    rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
    std::shared_ptr<SessionType> session_;
};
```

## Impact on Entity Classes

### Publisher, Subscriber, Service

- Rename `ContextType` template parameter to `SessionType`
- Replace `static_assert(is_base_of<Context, ...>)` with `static_assert(is_base_of<Session, ...>)`
- Store `weak_ptr<SessionType>` (unchanged pattern)
- Entity factories use `session->node` to access the lifecycle node for entity creation

### Timer

- `create_timer()` / `create_wall_timer()` use `session->node` instead of `context->node`
- Timers are still pushed to `session->timers` vector

### Action Server

- Currently takes a raw `LifecycleNode*` -- can use `&session->node` since session holds a reference

## Impact on Code Generator

- Rename `Context` to `Session` throughout Jinja2 template and Python generator
- `create_entities()` becomes `create_session()` -- same body but now constructs and returns a `shared_ptr<SessionType>` instead of populating a pre-existing context
- `name_expr` references change from `ctx->params.X` to `session->params.X`
- Entity creation uses `session->node` instead of `ctx->node`

## Alternatives Considered

### In-place reset with node protection
We explored making `node` survive in-place reset by pulling it into a separate "permanent" section of the struct. This added complexity (shadowed members, nested State structs) without addressing the fundamental issue that in-place reset can't atomically preserve one member while clearing the rest.

### Nullable shared_ptr with node plumbed separately
We explored removing `node` from the context/session entirely and passing it as a separate `LifecycleNode&` parameter to every callback. This worked but was verbose, and once we committed to create-and-destroy (no in-place reset), the motivation for separating node disappeared -- a reference inside Session is safe because the session is never mutated in-place.

### Eliminating `this` capture via NodeCore / inner handle
We explored moving lifecycle state into a shared inner object to avoid capturing `this` in lifecycle lambdas. This created reference cycle concerns (NodeCore <-> LifecycleNode) and added indirection without solving the fundamental issue: with composition + virtual methods, the lambdas need to reach the vtable somehow.

### Inheritance from LifecycleNode
Would eliminate the `this` capture problem entirely (lifecycle callbacks become virtual overrides, no lambdas). Rejected because it exposes lifecycle node internals to user code, violating the design goal of separating node dynamics from user code. The session is the intended bridge between the two.
