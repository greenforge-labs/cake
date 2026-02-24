#include <memory>

#include <cake_example/my_node_interface.hpp>

namespace cake_example::my_node {

struct Session : MyNodeSession<Session> {
    using MyNodeSession::MyNodeSession; // required — inherited constructors aren't transitive in C++
    int very_important_number = 5;
    int count = 0;
};

CallbackReturn on_configure(std::shared_ptr<Session> sn);

// IMPORTANT - this _must_ match the node name. Cake expects the node to be defined at pkg_name::node_name::NodeName
using MyNode = MyNodeBase<Session, on_configure>;

} // namespace cake_example::my_node
