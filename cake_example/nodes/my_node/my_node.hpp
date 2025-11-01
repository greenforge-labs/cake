#include <memory>

#include <cake_example/my_node_interface.hpp>

namespace cake_example::my_node {

struct Context : MyNodeContext<Context> {
    int very_important_number = 5;
};

void init(std::shared_ptr<Context> ctx);

using MyNode = MyNodeBase<Context, init>;

} // namespace cake_example::my_node
