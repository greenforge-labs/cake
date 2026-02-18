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
- **BaseNode**: Direct inheritance from `LifecycleNode` (no composition, no `LifecycleNodeImpl` wrapper). The move from constructor-based init to `on_configure` makes `shared_from_this()` available, which was the original blocker for inheritance.
- **Only lifecycle nodes**: No flag/option to use regular `rclcpp::Node`.
- **Weak pointers in wrappers**: Entity wrappers (Publisher, Subscriber, Service) and timer/thread callbacks store `weak_ptr<Context>` instead of `shared_ptr<Context>`. This eliminates reference cycles between the context and its entities, making cleanup trivial (`ctx_.reset()` frees everything) and ensuring callbacks become graceful no-ops if the context is destroyed.

### User callback model

Cake owns the ROS entity lifecycle — creating, activating, deactivating, and destroying publishers, subscribers, services, timers, and action servers. The user callbacks are for managing custom state and external resources that cake doesn't know about.

- **`on_configure`** (required): The main user callback. Wire up subscriber/service callbacks, create timers, initialize custom context fields (load models, open files, set up algorithms). This replaces the old `init`.
- **`on_activate`** / **`on_deactivate`** (optional): Most users leave these as defaults. Useful for connecting/disconnecting external systems, arming/disarming hardware, or managing custom state transitions. User callbacks always run first: `on_activate` runs before publishers are activated and timers started (so publishing is not yet available), `on_deactivate` runs before they are torn down (so publishing still works). This means `on_activate` failures require no rollback — nothing has been activated yet.
- **`on_cleanup`** (optional): Release resources not covered by RAII. If everything on the context has proper destructors, this can be left as the default — `destroy_context_()` handles the rest.
- **`on_shutdown`** (optional, void return): Called before the framework tears down on shutdown. From Active, all ROS resources are still live — publishers work, timers are running — so the user can publish a final status, send a last command to hardware, etc. From Inactive, the context exists but publishers won't publish; useful for non-ROS cleanup (close connections, release hardware). From Unconfigured, the callback is skipped (no context). Cannot block shutdown.

If all custom state lives on the context and uses RAII, only `on_configure` needs a user implementation.

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

    try {
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

        auto result = on_configure_func(ctx_);
        if (result == CallbackReturn::FAILURE) {
            ctx_.reset();  // FAILURE: back to Unconfigured, clean slate
        }
        // ERROR: leave ctx_ for on_error to clean up (consistent with all other transitions)
        return result;
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "on_configure failed: %s", e.what());
        return CallbackReturn::ERROR;
    }
}
```

Note: `for_each_param` loops remain the same, just moved from the constructor into `on_configure`.

#### New: `on_activate` override

```cpp
CallbackReturn on_activate(const rclcpp_lifecycle::State &) override {
    try {
        auto result = on_activate_func(ctx_);
        if (result != CallbackReturn::SUCCESS) {
            return result;  // nothing activated yet, nothing to roll back
        }

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

        return result;
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "on_activate failed: %s", e.what());
        return CallbackReturn::ERROR;
    }
}
```

#### New: `on_deactivate` override

```cpp
CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override {
    try {
        auto result = on_deactivate_func(ctx_);
        if (result != CallbackReturn::SUCCESS) {
            // node stays Active — don't tear down anything
            return result;
        }

        deactivate_entities_();
        return result;
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "on_deactivate failed: %s", e.what());
        return CallbackReturn::ERROR;
    }
}
```

#### New: `on_cleanup` override

Destroy the context entirely. Because all wrappers and timer callbacks hold `weak_ptr<Context>` (not `shared_ptr`), there are no reference cycles. Dropping `ctx_` frees everything cleanly:

```cpp
CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override {
    try {
        auto result = on_cleanup_func(ctx_);
        if (result == CallbackReturn::SUCCESS) {
            destroy_context_();
        }
        return result;
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "on_cleanup failed: %s", e.what());
        return CallbackReturn::ERROR;
    }
}
```

After `ctx_` is reset, any in-flight callbacks that try to `lock()` their `weak_ptr` will get `nullptr` and gracefully no-op. A fresh context is created on the next `on_configure`.

#### Exception handling

Every generated lifecycle override wraps its entire body in `try/catch (const std::exception &e)`. If either the framework code (entity creation, activation, teardown) or the user callback throws, the exception is caught, logged via `RCLCPP_ERROR`, and `CallbackReturn::ERROR` is returned. This sends the node to ErrorProcessing → `on_error` → `destroy_context_()` → Finalized.

This is deliberate: we don't rely on rclcpp's internal exception handling around transition callbacks (which is underdocumented and may vary between versions). The generated code handles it explicitly and predictably.

In `on_configure`, an exception during entity creation naturally skips the user callback (it comes after). In the other overrides, the user callback runs first — if it throws, the framework teardown is skipped, but `on_error` handles full cleanup regardless.

#### New: `on_shutdown` override

Called when the node is shut down from any primary state (Unconfigured, Inactive, Active). This is a direct path to Finalized — it does **not** go through `on_deactivate` or `on_cleanup`. The generated override handles graceful teardown by reusing the same helpers. The user's `on_shutdown_func` has `void` return because shutdown cannot be blocked.

```cpp
CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override {
    if (ctx_) {
        on_shutdown_func(ctx_);
    }

    if (ctx_ && state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
        deactivate_entities_();
    }

    if (ctx_) {
        destroy_context_();
    }

    return CallbackReturn::SUCCESS;
}
```

Note: `on_deactivate_func` and `on_cleanup_func` are **not** called during shutdown — the user gets `on_shutdown_func` as their single hook for this path. Each transition calls exactly one user callback.

#### New: `on_error` override

Called when any transition callback returns `CallbackReturn::ERROR` (not `FAILURE` — that simply bounces back to the previous primary state). Cleans up the context and returns `FAILURE`, which transitions the node to **Finalized** (terminal). `ERROR` is unrecoverable — the node cannot be reconfigured. No user callback.

```cpp
CallbackReturn on_error(const rclcpp_lifecycle::State &) override {
    if (ctx_) {
        destroy_context_();
    }
    return CallbackReturn::FAILURE;  // → Finalized
}
```

#### Private helpers

Shared teardown logic used by `on_deactivate`, `on_cleanup`, `on_shutdown`, and `on_error`:

```cpp
private:
    void deactivate_entities_() {
        for (auto &t : ctx_->timers) { t->cancel(); }

        {%- for pub in publishers %}
        {%- if pub.for_each_param %}
        for (auto &[key, pub] : ctx_->publishers.{{ pub.field_name }}) { pub->deactivate(); }
        {%- else %}
        ctx_->publishers.{{ pub.field_name }}->deactivate();
        {%- endif %}
        {%- endfor %}
    }

    void destroy_context_() {
        ctx_.reset();
    }
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
- `on_cleanup` calls the user's `on_cleanup_func`, then calls `destroy_context_()` (only if the user returned `SUCCESS`). That single call frees the entire context and all its entities. This works for timers because rclcpp's `CallbackGroup` stores `weak_ptr<TimerBase>` (not `shared_ptr`), so `ctx_->timers` holds the only owning references. When the context is destroyed, the timers are destroyed, and the callback group's weak pointers expire naturally — no explicit removal needed.
- `on_shutdown` also calls `destroy_context_()`, reusing the same path. Shutdown always succeeds — it cannot be blocked.
- Any in-flight callbacks (timers, subscribers) that fire after cleanup gracefully no-op — their `weak_ptr::lock()` returns `nullptr`.
- `on_configure` creates a fresh `ctx_` via `std::make_shared<ContextType>()` and repopulates everything from scratch.
- User state (e.g. `int counter = 0` on the derived context) is automatically reset to its default value on reconfigure — no user action needed.

This should be tested explicitly.

## Self-triggered transitions

User code can trigger lifecycle transitions from within callbacks (e.g. `ctx->node->deactivate()` from a timer or subscriber callback to take the node offline when an error is detected). The transition runs synchronously — the generated `on_deactivate` / `on_shutdown` fires inline. The design handles this gracefully:

- The currently-executing callback is not interrupted — it runs to completion.
- Timers are cancelled (no future firings) and publishers become no-ops for future publishes, but nothing crashes.
- On shutdown, `destroy_context_()` resets `ctx_`, but the callback holds a local `shared_ptr` (from `weak_ptr::lock()`), so the context stays alive until the callback returns.
- Code after the transition call continues in a degraded state (publishes silently dropped, etc.). This is expected — the caller asked to deactivate.

There is no programmatic way to enter the ErrorProcessing state. `on_error` is only triggered when a transition callback returns `CallbackReturn::ERROR`.

## Transition reference

### Configure: Unconfigured → Inactive

| Step | What happens | `ctx_` state |
|---|---|---|
| Entry | `ctx_` is null | null |
| `on_configure` | Create `ctx_`, populate entities (params, pubs, subs, services, actions) | exists, entities created |
| | Call `on_configure_func(ctx_)` | |
| **SUCCESS** | → **Inactive**. Entities exist but not activated. Publishers won't publish, timers not started | exists, inactive |
| **FAILURE** | `ctx_.reset()` → **Unconfigured**. Rolled back to clean slate | null |
| **ERROR** | `ctx_` left as-is. `on_error` fires, `destroy_context_()` → **Finalized** | null |
| **exception** | Caught, logged. Returns ERROR — same path as above | null |

### Activate: Inactive → Active

| Step | What happens | `ctx_` state |
|---|---|---|
| Entry | Entities exist but inactive | exists, inactive |
| `on_activate` | Call `on_activate_func(ctx_)` — node is activating, not yet Active. Publishers are not yet activated, timers not yet started. User callback is for custom state setup (connect to external systems, arm hardware, etc.). Timers created here are picked up by the reset loop that follows | exists, inactive |
| **SUCCESS** | Activate publishers, reset (start) all timers (including any created by the user callback) → **Active** | exists, active |
| **FAILURE** | Nothing was activated, nothing to roll back → **Inactive** | exists, inactive |
| **ERROR** | Nothing was activated. `on_error` fires, `destroy_context_()` → **Finalized** | null |
| **exception** | Caught, logged. Returns ERROR — same path as above | null |

### Deactivate: Active → Inactive

| Step | What happens | `ctx_` state |
|---|---|---|
| Entry | Everything is live | exists, active |
| `on_deactivate` | Call `on_deactivate_func(ctx_)` first | |
| **SUCCESS** | `deactivate_entities_()`: cancel timers, deactivate publishers → **Inactive** | exists, inactive |
| **FAILURE** | Return early, everything stays live → **Active** | exists, active |
| **ERROR** | Return early, nothing torn down. `on_error` fires, `destroy_context_()` → **Finalized** | null |
| **exception** | Caught, logged. Returns ERROR — same path as above | null |

### Cleanup: Inactive → Unconfigured

| Step | What happens | `ctx_` state |
|---|---|---|
| Entry | Entities exist but inactive | exists, inactive |
| `on_cleanup` | Call `on_cleanup_func(ctx_)` first | |
| **SUCCESS** | `destroy_context_()` → **Unconfigured**. Ready for fresh configure | null |
| **FAILURE** | Context preserved → **Inactive** | exists, inactive |
| **ERROR** | Context preserved. `on_error` fires, `destroy_context_()` → **Finalized** | null |
| **exception** | Caught, logged. Returns ERROR — same path as above | null |

### Shutdown from Unconfigured: Unconfigured → Finalized

| Step | What happens | `ctx_` state |
|---|---|---|
| `on_shutdown` | `ctx_` is null — all `if (ctx_)` guards skip. No user callback called | null |
| | → **Finalized** | null |

### Shutdown from Inactive: Inactive → Finalized

| Step | What happens | `ctx_` state |
|---|---|---|
| `on_shutdown` | `on_shutdown_func(ctx_)` called | exists |
| | State is not Active — `deactivate_entities_()` skipped | exists |
| | `destroy_context_()` → **Finalized** | null |

### Shutdown from Active: Active → Finalized

| Step | What happens | `ctx_` state |
|---|---|---|
| `on_shutdown` | `on_shutdown_func(ctx_)` called | exists, active |
| | State IS Active — `deactivate_entities_()`: cancel timers, deactivate publishers | exists, inactive |
| | `destroy_context_()` → **Finalized** | null |

### Error recovery: ErrorProcessing → Finalized

| Step | What happens | `ctx_` state |
|---|---|---|
| `on_error` | If `ctx_` exists: `destroy_context_()`. If already null (e.g. from `on_configure` rollback): no-op | null |
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
