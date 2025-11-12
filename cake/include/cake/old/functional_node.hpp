/***
USAGE:

.hpp

```cpp
namespace my_package {

void init(rclcpp::Node::SharedPtr node);
using MyNode = cake::FunctionalNode<"node_name", init>;

}
```

.cpp

```cpp
namespace my_package {

void init(rclcpp::Node::SharedPtr node) {
    // some implementation...
}

}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(my_package::MyNode);
```

***/
#pragma once

#pragma message(                                                                                                               \
    "WARNING: you are using the old deprecated version of cake. Please use the interface.yaml code generation system instead." \
)

#include <rclcpp/rclcpp.hpp>

#include "fixed_string.hpp"

namespace cake {

template <
    fixed_string node_name,
    auto init_func,
    auto extend_options = [](rclcpp::NodeOptions options) { return options; }>
class FunctionalNode {
  public:
    explicit FunctionalNode(const rclcpp::NodeOptions &options)
        : node_(std::make_shared<rclcpp::Node>(node_name.c_str(), extend_options(options))) {
        init_func(node_);
    }

    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() const {
        return this->node_->get_node_base_interface();
    }

  private:
    rclcpp::Node::SharedPtr node_;
};

} // namespace cake
