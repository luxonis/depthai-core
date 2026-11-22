#include <stdexcept>

#include "depthai/pipeline/ThreadedHostNode.hpp"
#include "depthai/pipeline/node/host/HostNode.hpp"
#include "depthai_pybind11_tests.hpp"

namespace {
// Deliberately holds only a C++ shared_ptr: no py::object or keep_alive policy
// may mask a failure to preserve the Python subclass in the shared_ptr caster.
struct NodeOwner {
    std::shared_ptr<dai::node::ThreadedHostNode> node;
    explicit NodeOwner(std::shared_ptr<dai::node::ThreadedHostNode> value) : node(std::move(value)) {}
};

struct BaseNodeOwner {
    std::shared_ptr<dai::Node> node;
    explicit BaseNodeOwner(std::shared_ptr<dai::Node> value) : node(std::move(value)) {}
};
}  // namespace

TEST_SUBMODULE(node_holder, m) {
    py::module_::import("depthai");
    py::class_<NodeOwner>(m, "NodeOwner")
        .def(py::init<std::shared_ptr<dai::node::ThreadedHostNode>>())
        .def("get", [](const NodeOwner& owner) { return owner.node; })
        .def("on_start", [](const NodeOwner& owner) { owner.node->onStart(); })
        .def("run", [](const NodeOwner& owner) { owner.node->run(); })
        .def("from_shared_from_this", [](const NodeOwner& owner) {
            return NodeOwner(std::static_pointer_cast<dai::node::ThreadedHostNode>(owner.node->shared_from_this()));
        })
        .def("shares_shared_from_this_owner", [](const NodeOwner& owner) {
            auto other = owner.node->shared_from_this();
            return !owner.node.owner_before(other) && !other.owner_before(owner.node);
        })
        .def("process_group", [](const NodeOwner& owner) {
            auto host = std::dynamic_pointer_cast<dai::node::HostNode>(owner.node);
            if(!host) throw std::invalid_argument("Expected a HostNode");
            return host->processGroup(std::make_shared<dai::MessageGroup>());
        });
    py::class_<BaseNodeOwner>(m, "BaseNodeOwner")
        .def(py::init<std::shared_ptr<dai::Node>>())
        .def("get", [](const BaseNodeOwner& owner) { return owner.node; });
}
