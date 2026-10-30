#include "Common.hpp"
#include "NodeBindings.hpp"
#include "depthai/pipeline/node/host/Display.hpp"

void bind_display(pybind11::module& m, void* pCallstack) {
#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    auto node = addNode<dai::node::Display, dai::node::ThreadedHostNode>("Display", DOC(dai, node, Display));
#endif
    auto* callstack = static_cast<Callstack*>(pCallstack);
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    node.def(py::init([](std::string name) { return getImplicitPipeline()->create<dai::node::Display>(std::move(name)); }),
             py::arg("name") = "Display",
             DOC(dai, node, Display, Display))
        .def_readonly("input", &dai::node::Display::input, DOC(dai, node, Display, input));
#endif
}
