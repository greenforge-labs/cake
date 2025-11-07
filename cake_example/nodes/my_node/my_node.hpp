#include <memory>

#include <cake_example/my_node_interface.hpp>

namespace cake_example::my_node {

struct Context : MyNodeContext<Context> {
    int very_important_number = 5;
    int count = 0;
};

void init(std::shared_ptr<Context> ctx);

// IMPORTANT - this _must_ match the node name. Cake expects the node to be defined at pkg_name::node_name::NodeName
using MyNode = MyNodeBase<Context, init>;

} // namespace cake_example::my_node
