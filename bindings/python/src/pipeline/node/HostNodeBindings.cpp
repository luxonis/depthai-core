#include "NodeBindings.hpp"
#include "Common.hpp"
#include "depthai/pipeline/HostNode.hpp"
#include "depthai/pipeline/node/host/SyncedNode.hpp"

using namespace dai;
using namespace dai::node;

class PyHostNode : public NodeCRTP<HostNode, PyHostNode> {
public:
    void run() override {
        PYBIND11_OVERRIDE_PURE(
                void,
                HostNode,
                run);
    }
};

class PySyncedNode : public NodeCRTP<SyncedNode, PySyncedNode> {
public:
    std::shared_ptr<Buffer> runOnce(std::shared_ptr<dai::MessageGroup> in) override {
        PYBIND11_OVERRIDE_PURE(
                std::shared_ptr<Buffer>,
                SyncedNode,
                runOnce);
    }
};

void bind_hostnode(pybind11::module& m, void* pCallstack){
    // declare upfront
    auto hostNode = py::class_<HostNode, PyHostNode, ThreadedNode, std::shared_ptr<HostNode>>(m, "HostNode", DOC(dai, HostNode));
    auto syncedNode = py::class_<SyncedNode, PySyncedNode, HostNode, std::shared_ptr<SyncedNode>>(m, "SyncedNode", DOC(dai, node, SyncedNode));

    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    // Call the rest of the type defines, then perform the actual bindings
    Callstack* callstack = (Callstack*) pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);
    // Actual bindings
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////

    hostNode
        .def(py::init<>())
        .def("run", &HostNode::run)
    ;

    syncedNode
        .def(py::init<>())
        .def("runOnce", &SyncedNode::runOnce)
        .def_property_readonly("inputs", [](SyncedNode &node){return node.inputs;})
    ;
}
