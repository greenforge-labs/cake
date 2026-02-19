# Cake Lifecycle Node Migration — Design Document

## Overview

Replace `rclcpp::Node` with `rclcpp_lifecycle::LifecycleNode` throughout the cake framework. This is a breaking change. After migration, all cake nodes are lifecycle nodes with the standard state machine: **Unconfigured -> Inactive -> Active -> Finalized**.

### Key decisions

- **User callback API**: Template parameters. All lifecycle callbacks return `CallbackReturn`. `on_configure` is required (replaces `init`), `on_activate` / `on_deactivate` / `on_cleanup` are optional with default `SUCCESS`. `on_shutdown` is optional with `void` return (cannot block shutdown).
- **Shutdown & error**: Generated `on_shutdown` calls the user's `on_shutdown_func`, then does graceful teardown (deactivate entities if Active, destroy context). Teardown logic is shared with `on_deactivate` / `on_cleanup` via private helpers. Generated `on_error` cleans up and goes to Finalized — `ERROR` is treated as unrecoverable.
- **Subscribers**: Auto-drop messages when node is not Active.
- **Services**: Auto-reject (default-constructed response) when node is not Active.
- **Action servers**: Auto-reject goals when node is not Active.
- **Timers**: Created inactive when node is not Active, callback-guarded for safety.
- **Publishers**: Use `LifecyclePublisher` — publishing is a no-op when not Active. Generated code handles activate/deactivate.
- **BaseNode**: Composition — wraps a `LifecycleNode::SharedPtr` member, same pattern as today. Lifecycle callbacks are registered via `register_on_configure()` etc. on the wrapped node. The context object is the central driving object — all state lives on the context, not scattered across the node class. No `shared_from_this()` needed; `ctx->node` is assigned directly from the `node_` member.
- **Only lifecycle nodes**: No flag/option to use regular `rclcpp::Node`.
- **Weak pointers in wrappers**: Entity wrappers (Publisher, Subscriber, Service) and timer/thread callbacks store `weak_ptr<Context>` instead of `shared_ptr<Context>`. This eliminates reference cycles between the context and its entities, making cleanup trivial (`ctx->reset()` frees everything) and ensuring callbacks become graceful no-ops if the context is destroyed.

### User callback model

Cake owns the ROS entity lifecycle — creating, activating, deactivating, and destroying publishers, subscribers, services, timers, and action servers. The user callbacks are for managing custom state and external resources that cake doesn't know about.

- **`on_configure`** (required): The main user callback. Wire up subscriber/service callbacks, create timers, initialize custom context fields (load models, open files, set up algorithms). This replaces the old `init`.
- **`on_activate`** / **`on_deactivate`** (optional): Most users leave these as defaults. Useful for connecting/disconnecting external systems, arming/disarming hardware, or managing custom state transitions. User callbacks always run first: `on_activate` runs before publishers are activated and timers started (so publishing is not yet available), `on_deactivate` runs before they are torn down (so publishing still works). This means `on_activate` failures require no rollback — nothing has been activated yet.
- **`on_cleanup`** (optional): Release resources not covered by RAII. If everything on the context has proper destructors, this can be left as the default — `ctx->reset()` handles the rest.
- **`on_shutdown`** (optional, void return): Called before the framework tears down on shutdown. From Active, publishers are still activated — the user can publish a final status or send a last command to hardware. Timer callbacks will not fire (they guard on `PRIMARY_STATE_ACTIVE`, which is false during the shutdown transition). From Inactive, the context exists but publishers won't publish; useful for non-ROS cleanup (close connections, release hardware). From Unconfigured, the callback is skipped (no context). Cannot block shutdown.

If all custom state lives on the context and uses RAII, only `on_configure` needs a user implementation.

### Entity behavior during lifecycle transitions

During transitional states (configuring, activating, deactivating, shutting down), the node's primary state is not `PRIMARY_STATE_ACTIVE`. Entity types differ in how they behave during these windows:

- **Publishers**: Controlled by explicit `activate()` / `deactivate()` calls (via `LifecyclePublisher`), not by node state checks. They remain functional during transitions from Active until `deactivate_entities()` runs. This means the user can publish from `on_deactivate` and `on_shutdown` callbacks.
- **Timers, Subscribers, Services, Action servers**: Guarded by a `PRIMARY_STATE_ACTIVE` check in their callback wrappers. Callbacks are no-ops / requests are rejected during any transitional state, even if the transition started from Active.

This is an intentional asymmetry. Publishers are outbound — the node decides when to publish, and being able to send a final message during `on_shutdown` or `on_deactivate` is valuable. Timers, subscribers, services, and action servers involve accepting inbound work or executing scheduled user logic; allowing them to fire during teardown risks running user code in a partially-torn-down state. Keeping them guarded on `PRIMARY_STATE_ACTIVE` is the safer default.

### `CallbackReturn` semantics

All user lifecycle callbacks (except `on_shutdown`) return `CallbackReturn`. The three values have distinct meanings:

- **`SUCCESS`** — Transition proceeds normally to the target state.
- **`FAILURE`** — Transition is rolled back to the previous primary state. The node can retry. Use this for transient problems: a config file isn't available yet, a hardware connection timed out, an external dependency isn't ready. The node stays in a valid state and a lifecycle manager (or user code) can attempt the transition again later.
- **`ERROR`** — Unrecoverable error. The node enters ErrorProcessing, `on_error` fires (cleans up the context), and the node goes to **Finalized** (terminal). Use this when the node is in a state that cannot be recovered from: corrupted internal state, hardware fault, invariant violation. The node cannot be reconfigured — it must be destroyed and recreated.

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

Keep composition — `BaseNode` wraps a `LifecycleNode::SharedPtr`, same pattern as the current `rclcpp::Node` wrapper:

```cpp
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "fixed_string.hpp"

namespace cake {

template <fixed_string node_name, auto extend_options = [](rclcpp::NodeOptions options) { return options; }>
class BaseNode {
  public:
    explicit BaseNode(const rclcpp::NodeOptions &options)
        : node_(std::make_shared<rclcpp_lifecycle::LifecycleNode>(node_name.c_str(), extend_options(options))) {}

    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() const {
        return node_->get_node_base_interface();
    }

  protected:
    rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
};

} // namespace cake
```

Notes:
- `get_node_base_interface()` delegates to the wrapped node — same as the current non-lifecycle implementation.
- `RCLCPP_COMPONENTS_REGISTER_NODE` continues to work since `get_node_base_interface()` provides the required interface.
- The `protected: node_` member changes type from `rclcpp::Node::SharedPtr` to `rclcpp_lifecycle::LifecycleNode::SharedPtr`. The generated `*Base` class assigns `ctx->node = node_` directly — no `shared_from_this()` needed.
- The generated class registers lifecycle callbacks via `node_->register_on_configure(...)` etc. instead of overriding virtual methods.

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

The `using CallbackReturn` alias is emitted at namespace scope in the generated header, before the class definition, so it's available in the template parameter defaults.

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

The generated class has **no private members and no private methods**. The constructor is the entire class — it sets up the shared context holder, defines helper lambdas, and registers all lifecycle callbacks on the wrapped node. All lifecycle state flows through `ctx`, which is owned by the closures, not the class.

`ctx` is a `shared_ptr<shared_ptr<ContextType>>` — the outer pointer provides shared ownership across all the lambdas; the inner pointer is the actual context (null when unconfigured). `node_` is captured as a `weak_ptr` to avoid a reference cycle (node → registered callbacks → node).

```cpp
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

explicit {{ class_name }}Base(const rclcpp::NodeOptions &options)
    : cake::BaseNode<"{{ node_name }}", extend_options>(options) {
    static_assert(
        std::is_base_of_v<{{ class_name }}Context<ContextType>, ContextType>,
        "ContextType must be a child of {{ class_name }}Context"
    );

    auto ctx = std::make_shared<std::shared_ptr<ContextType>>();
    std::weak_ptr<rclcpp_lifecycle::LifecycleNode> weak_node = node_;

    auto deactivate_entities = [ctx]() { /* shown below */ };

    node_->register_on_configure([ctx, weak_node](const auto &) -> CallbackReturn { /* shown below */ });
    node_->register_on_activate([ctx](const auto &) -> CallbackReturn { /* shown below */ });
    node_->register_on_deactivate([ctx, deactivate_entities](const auto &) -> CallbackReturn { /* shown below */ });
    node_->register_on_cleanup([ctx](const auto &) -> CallbackReturn { /* shown below */ });
    node_->register_on_shutdown([ctx, deactivate_entities](const auto &state) -> CallbackReturn { /* shown below */ });
    node_->register_on_error([ctx, deactivate_entities](const auto &) -> CallbackReturn { /* shown below */ });
}
```

Note: no lambda captures `this`. The only things captured are `ctx` (shared context holder), `weak_node` (for configure only), and `deactivate_entities` (shared teardown helper).

#### `deactivate_entities` lambda

Local helper lambda shared (via capture) by the deactivate, shutdown, and error callbacks. Idempotent — safe to call even if entities were never activated:

```cpp
auto deactivate_entities = [ctx]() {
    for (auto &t : (*ctx)->timers) { t->cancel(); }

    {%- for pub in publishers %}
    {%- if pub.for_each_param %}
    for (auto &[key, pub] : (*ctx)->publishers.{{ pub.field_name }}) { pub->deactivate(); }
    {%- else %}
    (*ctx)->publishers.{{ pub.field_name }}->deactivate();
    {%- endif %}
    {%- endfor %}
};
```

#### Registered: `on_configure`

```cpp
node_->register_on_configure([ctx, weak_node](const auto &) -> CallbackReturn {
    auto node = weak_node.lock();
    *ctx = std::make_shared<ContextType>();
    (*ctx)->node = node;

    // init parameters
    (*ctx)->param_listener = std::make_shared<ParamListener>((*ctx)->node);
    (*ctx)->params = (*ctx)->param_listener->get_params();

    // init publishers
    {%- for pub in publishers %}
    (*ctx)->publishers.{{ pub.field_name }} = cake::create_publisher<{{ pub.msg_type }}>(*ctx, {{ pub.name_expr }}, {{ pub.qos_code }});
    {%- endfor %}

    // init subscribers
    {%- for sub in subscribers %}
    (*ctx)->subscribers.{{ sub.field_name }} = cake::create_subscriber<{{ sub.msg_type }}>(*ctx, {{ sub.name_expr }}, {{ sub.qos_code }});
    {%- endfor %}

    // init services
    {%- for srv in services %}
    (*ctx)->services.{{ srv.field_name }} = cake::create_service<{{ srv.service_type }}>(*ctx, {{ srv.name_expr }});
    {%- endfor %}

    // init service clients
    {%- for client in service_clients %}
    (*ctx)->service_clients.{{ client.field_name }} = (*ctx)->node->template create_client<{{ client.service_type }}>({{ client.name_expr }});
    {%- endfor %}

    // init actions
    {%- for action in actions %}
    (*ctx)->actions.{{ action.field_name }} = cake::create_single_goal_action_server<{{ action.action_type }}>(*ctx, {{ action.name_expr }});
    {%- endfor %}

    // init action clients
    {%- for client in action_clients %}
    (*ctx)->action_clients.{{ client.field_name }} = rclcpp_action::create_client<{{ client.action_type }}>((*ctx)->node, {{ client.name_expr }});
    {%- endfor %}

    auto result = on_configure_func(*ctx);
    if (result == CallbackReturn::FAILURE) {
        ctx->reset();  // FAILURE: back to Unconfigured, clean slate
    }
    // ERROR: leave ctx for on_error to clean up (consistent with all other transitions)
    return result;
});
```

Note: `for_each_param` loops remain the same, just moved from the old constructor into the configure callback. `weak_node.lock()` always succeeds — this callback only runs while the lifecycle node is alive.

#### Registered: `on_activate`

```cpp
node_->register_on_activate([ctx](const auto &) -> CallbackReturn {
    auto result = on_activate_func(*ctx);
    if (result != CallbackReturn::SUCCESS) {
        return result;  // nothing activated yet, nothing to roll back
    }

    // activate lifecycle publishers
    {%- for pub in publishers %}
    {%- if pub.for_each_param %}
    for (auto &[key, pub] : (*ctx)->publishers.{{ pub.field_name }}) { pub->activate(); }
    {%- else %}
    (*ctx)->publishers.{{ pub.field_name }}->activate();
    {%- endif %}
    {%- endfor %}

    // reset all timers
    for (auto &t : (*ctx)->timers) { t->reset(); }

    return result;
});
```

#### Registered: `on_deactivate`

```cpp
node_->register_on_deactivate([ctx, deactivate_entities](const auto &) -> CallbackReturn {
    auto result = on_deactivate_func(*ctx);
    if (result != CallbackReturn::SUCCESS) {
        // node stays Active — don't tear down anything
        return result;
    }

    deactivate_entities();
    return result;
});
```

#### Registered: `on_cleanup`

Destroy the context entirely. Because all entity wrappers and timer callbacks hold `weak_ptr<Context>` (not `shared_ptr`), there are no reference cycles. Resetting the inner pointer frees everything cleanly:

```cpp
node_->register_on_cleanup([ctx](const auto &) -> CallbackReturn {
    auto result = on_cleanup_func(*ctx);
    if (result == CallbackReturn::SUCCESS) {
        ctx->reset();
    }
    return result;
});
```

After the inner pointer is reset, any in-flight callbacks that try to `lock()` their `weak_ptr` will get `nullptr` and gracefully no-op. A fresh context is created on the next configure.

#### Exception handling

None of the registered callbacks use try/catch. rclcpp handles this for us.

rclcpp's `execute_callback` (in `lifecycle_node_interface_impl.cpp`) wraps every transition callback in `try/catch (const std::exception &)`. If a registered callback or the user's callback throws a `std::exception`:

1. rclcpp catches it and logs the error.
2. The return code is set to `CallbackReturn::ERROR`.
3. The state machine enters ErrorProcessing, which invokes the registered `on_error` callback.
4. `on_error` deactivates entities and destroys the context → node goes to **Finalized**.

This is sufficient because the `on_error` callback handles teardown from any prior state — `deactivate_entities` is idempotent (safe to call even if entities were never activated), and `ctx->reset()` frees the rest. There is no case where we need our own catch to run intermediate teardown before `on_error`.

Note: only `std::exception` (and subclasses) are caught. A non-`std::exception` throw (e.g. `throw 42`) propagates uncaught and terminates the process. This is standard C++ — don't do that.

#### Registered: `on_shutdown`

Called when the node is shut down from any primary state (Unconfigured, Inactive, Active). This is a direct path to Finalized — it does **not** go through `on_deactivate` or `on_cleanup`. The user's `on_shutdown_func` has `void` return because shutdown cannot be blocked.

```cpp
node_->register_on_shutdown([ctx, deactivate_entities](const auto &state) -> CallbackReturn {
    if (*ctx) {
        on_shutdown_func(*ctx);
    }

    if (*ctx && state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
        deactivate_entities();
    }

    if (*ctx) {
        ctx->reset();
    }

    return CallbackReturn::SUCCESS;
});
```

Note: `on_deactivate_func` and `on_cleanup_func` are **not** called during shutdown — the user gets `on_shutdown_func` as their single hook for this path. Each transition calls exactly one user callback.

#### Registered: `on_error`

Called when any transition callback returns `CallbackReturn::ERROR` (not `FAILURE` — that simply bounces back to the previous primary state). Deactivates entities and destroys the context, then returns `FAILURE`, which transitions the node to **Finalized** (terminal). `ERROR` is unrecoverable — the node cannot be reconfigured. No user callback. `deactivate_entities` is idempotent — safe to call regardless of whether entities were ever activated.

```cpp
node_->register_on_error([ctx, deactivate_entities](const auto &) -> CallbackReturn {
    if (*ctx) {
        deactivate_entities();
        ctx->reset();
    }
    return CallbackReturn::FAILURE;  // → Finalized
});
```

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

**`my_node.cpp`** — rename `init` to `on_configure`, return `CallbackReturn`:
```cpp
CallbackReturn on_configure(std::shared_ptr<Context> ctx) {
    // ... same body as current init() ...
    return CallbackReturn::SUCCESS;
}
```

Optionally add `on_activate` / `on_deactivate` / `on_shutdown` to demonstrate the full lifecycle.

---

## Summary of file changes

| File | Change |
|---|---|
| `context.hpp` | `rclcpp::Node::SharedPtr` -> `rclcpp_lifecycle::LifecycleNode::SharedPtr` |
| `base_node.hpp` | Wrap `LifecycleNode::SharedPtr` instead of `rclcpp::Node::SharedPtr`, register lifecycle callbacks |
| `publisher.hpp` | Use `LifecyclePublisher`, add `activate()` / `deactivate()`, `weak_ptr` context |
| `subscriber.hpp` | Lifecycle state check in callback, `weak_ptr` context |
| `service.hpp` | Lifecycle state check, auto-reject when inactive, `weak_ptr` context |
| `action_server.hpp` | Change `rclcpp::Node*` to `LifecycleNode*`, reject goals when inactive |
| `timer.hpp` | Callback guard, `autostart` / `cancel()` based on state, `weak_ptr` context |
| `thread.hpp` | Remove entirely |
| `fixed_string.hpp` | No change |
| `qos_helpers.hpp` | No change |
| `node_interface.hpp.jinja2` | Major rewrite: constructor registers lifecycle callbacks on wrapped node |
| `node_registration.cpp.jinja2` | No change |
| `generate_node_interface.py` | Minor change to template invocation (same template variables) |
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
- `on_cleanup` calls the user's `on_cleanup_func`, then calls `ctx->reset()` (only if the user returned `SUCCESS`). That single call frees the entire context and all its entities. This works for timers because rclcpp's `CallbackGroup` stores `weak_ptr<TimerBase>` (not `shared_ptr`), so `ctx->timers` holds the only owning references. When the context is destroyed, the timers are destroyed, and the callback group's weak pointers expire naturally — no explicit removal needed. (Verified in `callback_group.hpp:302` — `timer_ptrs_` is `std::vector<rclcpp::TimerBase::WeakPtr>`. The executor layer also uses `WeakPtr` in its collections; `shared_ptr` only appears transiently during dispatch.)
- `on_shutdown` also calls `ctx->reset()`, reusing the same path. Shutdown always succeeds — it cannot be blocked.
- Any in-flight callbacks (timers, subscribers) that fire after cleanup gracefully no-op — their `weak_ptr::lock()` returns `nullptr`.
- `on_configure` assigns a fresh `*ctx = std::make_shared<ContextType>()` and repopulates everything from scratch.
- User state (e.g. `int counter = 0` on the derived context) is automatically reset to its default value on reconfigure — no user action needed.

This should be tested explicitly.

## Self-triggered transitions

User code can trigger lifecycle transitions from within callbacks (e.g. `ctx->node->deactivate()` from a timer or subscriber callback to take the node offline when an error is detected). The transition runs synchronously — the registered `on_deactivate` / `on_shutdown` callback fires inline. The design handles this gracefully:

- The currently-executing callback is not interrupted — it runs to completion.
- Timers are cancelled (no future firings) and publishers become no-ops for future publishes, but nothing crashes.
- On shutdown, `ctx->reset()` clears the inner pointer, but the callback holds a local `shared_ptr` (from `weak_ptr::lock()`), so the context stays alive until the callback returns.
- Code after the transition call continues in a degraded state (publishes silently dropped, etc.). This is expected — the caller asked to deactivate.

There is no programmatic way to enter the ErrorProcessing state. `on_error` is only triggered when a transition callback returns `CallbackReturn::ERROR`.

## Transition reference

### Configure: Unconfigured → Inactive

| Step | What happens | `*ctx` state |
|---|---|---|
| Entry | `*ctx` is null | null |
| `on_configure` | Create context, populate entities (params, pubs, subs, services, actions) | exists, entities created |
| | Call `on_configure_func(*ctx)` | |
| **SUCCESS** | → **Inactive**. Entities exist but not activated. Publishers won't publish, timers not started | exists, inactive |
| **FAILURE** | `ctx->reset()` → **Unconfigured**. Rolled back to clean slate | null |
| **ERROR** | `*ctx` left as-is. `on_error` fires, `ctx->reset()` → **Finalized** | null |
| **exception** | Caught by rclcpp, logged, converted to ERROR — same path as above | null |

### Activate: Inactive → Active

| Step | What happens | `*ctx` state |
|---|---|---|
| Entry | Entities exist but inactive | exists, inactive |
| `on_activate` | Call `on_activate_func(*ctx)` — node is activating, not yet Active. Publishers are not yet activated, timers not yet started. User callback is for custom state setup (connect to external systems, arm hardware, etc.). Timers created here are picked up by the reset loop that follows | exists, inactive |
| **SUCCESS** | Activate publishers, reset (start) all timers (including any created by the user callback) → **Active** | exists, active |
| **FAILURE** | Nothing was activated, nothing to roll back → **Inactive** | exists, inactive |
| **ERROR** | Nothing was activated. `on_error` fires, `ctx->reset()` → **Finalized** | null |
| **exception** | Caught by rclcpp, logged, converted to ERROR — same path as above | null |

### Deactivate: Active → Inactive

| Step | What happens | `*ctx` state |
|---|---|---|
| Entry | Everything is live | exists, active |
| `on_deactivate` | Call `on_deactivate_func(*ctx)` first | |
| **SUCCESS** | `deactivate_entities()`: cancel timers, deactivate publishers → **Inactive** | exists, inactive |
| **FAILURE** | Return early, everything stays live → **Active** | exists, active |
| **ERROR** | Return early, nothing torn down. `on_error` fires, `ctx->reset()` → **Finalized** | null |
| **exception** | Caught by rclcpp, logged, converted to ERROR — same path as above | null |

### Cleanup: Inactive → Unconfigured

| Step | What happens | `*ctx` state |
|---|---|---|
| Entry | Entities exist but inactive | exists, inactive |
| `on_cleanup` | Call `on_cleanup_func(*ctx)` first | |
| **SUCCESS** | `ctx->reset()` → **Unconfigured**. Ready for fresh configure | null |
| **FAILURE** | Context preserved → **Inactive** | exists, inactive |
| **ERROR** | Context preserved. `on_error` fires, `ctx->reset()` → **Finalized** | null |
| **exception** | Caught by rclcpp, logged, converted to ERROR — same path as above | null |

### Shutdown from Unconfigured: Unconfigured → Finalized

| Step | What happens | `*ctx` state |
|---|---|---|
| `on_shutdown` | `*ctx` is null — all `if (*ctx)` guards skip. No user callback called | null |
| | → **Finalized** | null |

### Shutdown from Inactive: Inactive → Finalized

| Step | What happens | `*ctx` state |
|---|---|---|
| `on_shutdown` | `on_shutdown_func(*ctx)` called | exists |
| | State is not Active — `deactivate_entities()` skipped | exists |
| | `ctx->reset()` → **Finalized** | null |

### Shutdown from Active: Active → Finalized

| Step | What happens | `*ctx` state |
|---|---|---|
| `on_shutdown` | `on_shutdown_func(*ctx)` called | exists, active |
| | State IS Active — `deactivate_entities()`: cancel timers, deactivate publishers | exists, inactive |
| | `ctx->reset()` → **Finalized** | null |

### Error recovery: ErrorProcessing → Finalized

| Step | What happens | `*ctx` state |
|---|---|---|
| `on_error` | If `*ctx` exists: `deactivate_entities()` (idempotent), then `ctx->reset()`. If already null: no-op (defensive guard — in practice `*ctx` should always exist here) | null |
| | Returns FAILURE → **Finalized**. ERROR is unrecoverable — node must be destroyed and recreated | null |

### User callback summary

| Transition | User callback | Can block? |
|---|---|---|
| Configure | `on_configure_func` | Yes (FAILURE/ERROR) |
| Activate | `on_activate_func` | Yes (FAILURE/ERROR) |
| Deactivate | `on_deactivate_func` | Yes (FAILURE/ERROR) |
| Cleanup | `on_cleanup_func` | Yes (FAILURE/ERROR) |
| Shutdown | `on_shutdown_func` | No (void return) |
| Error | *(none)* | No |
