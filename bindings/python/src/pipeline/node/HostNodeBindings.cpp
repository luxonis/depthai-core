#include "Common.hpp"
#include "NodeBindings.hpp"
#include "depthai/pipeline/ThreadedHostNode.hpp"
#include "depthai/pipeline/node/host/HostNode.hpp"

extern py::handle daiNodeModule;

using namespace dai;
using namespace dai::node;

class PyThreadedHostNode : public NodeCRTP<ThreadedHostNode, PyThreadedHostNode> {
   public:
    void run() override {
        PYBIND11_OVERRIDE_PURE(void, ThreadedHostNode, run);
    }
};

class PyHostNode : public NodeCRTP<HostNode, PyHostNode> {
   public:
    std::shared_ptr<Buffer> runOnce(std::shared_ptr<dai::MessageGroup> in) override {
        PYBIND11_OVERRIDE_PURE(std::shared_ptr<Buffer>, HostNode, runOnce, in);
    }
};

void bind_hostnode(pybind11::module& m, void* pCallstack){
    // declare upfront
    auto threadedHostNode =
        py::class_<ThreadedHostNode, PyThreadedHostNode, ThreadedNode, std::shared_ptr<ThreadedHostNode>>(daiNodeModule, "ThreadedHostNode", DOC(dai, node, ThreadedHostNode));
    auto hostNode = py::class_<HostNode, PyHostNode, ThreadedHostNode, std::shared_ptr<HostNode>>(daiNodeModule, "HostNode", DOC(dai, node, HostNode));

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

    threadedHostNode.def(py::init<>()).def("run", &ThreadedHostNode::run);

    hostNode.def(py::init<>())
        .def("runOnce", &HostNode::runOnce)
        .def_property_readonly(
            "inputs", [](HostNode& node) { return &node.inputs; }, py::return_value_policy::reference_internal)
        .def("runSyncingOnHost", &HostNode::runSyncingOnHost, DOC(dai, node, HostNode, runSyncingOnHost))
        .def("runSyncingOnDevice", &HostNode::runSyncingOnDevice, DOC(dai, node, HostNode, runSyncingOnDevice));
}
