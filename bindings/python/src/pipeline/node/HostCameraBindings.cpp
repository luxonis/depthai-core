#include "Common.hpp"
#include "depthai/pipeline/node/host/HostCamera.hpp"

void bind_hostcamera(pybind11::module& m, void* pCallstack) {
#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    auto node = addNode<dai::node::HostCamera, dai::node::ThreadedHostNode>("HostCamera", DOC(dai, node, HostCamera));
#endif
    auto* callstack = static_cast<Callstack*>(pCallstack);
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    node.def_readonly("out", &dai::node::HostCamera::out, DOC(dai, node, HostCamera, out));
#endif
}
