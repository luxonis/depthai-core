#include "../Common.hpp"
#include "../NodeBindings.hpp"
#include "depthai/pipeline/Node.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/internal/XLinkIn.hpp"

extern py::handle daiNodeInternalModule;

void bind_xlinkin(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::node;
    using namespace dai::node::internal;

    // Node and Properties declare upfront
    py::class_<XLinkInProperties> xlinkInProperties(m, "XLinkInProperties", DOC(dai, internal, XLinkInProperties));
    auto xlinkIn = ADD_NODE_INTERNAL(XLinkIn);

    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    // Call the rest of the type defines, then perform the actual bindings
    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);
    // Actual bindings
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////

    // Properties
    xlinkInProperties.def_readwrite("streamName", &XLinkInProperties::streamName)
        .def_readwrite("maxDataSize", &XLinkInProperties::maxDataSize)
        .def_readwrite("numFrames", &XLinkInProperties::numFrames);

    // Node
    xlinkIn.def_readonly("out", &XLinkIn::out, DOC(dai, node, internal, XLinkIn, out))
        .def("setStreamName", &XLinkIn::setStreamName, py::arg("streamName"), DOC(dai, node, internal, XLinkIn, setStreamName))
        .def("setMaxDataSize", &XLinkIn::setMaxDataSize, py::arg("maxDataSize"), DOC(dai, node, internal, XLinkIn, setMaxDataSize))
        .def("setNumFrames", &XLinkIn::setNumFrames, py::arg("numFrames"), DOC(dai, node, internal, XLinkIn, setNumFrames))
        .def("getStreamName", &XLinkIn::getStreamName, DOC(dai, node, internal, XLinkIn, getStreamName))
        .def("getMaxDataSize", &XLinkIn::getMaxDataSize, DOC(dai, node, internal, XLinkIn, getMaxDataSize))
        .def("getNumFrames", &XLinkIn::getNumFrames, DOC(dai, node, internal, XLinkIn, getNumFrames));
    daiNodeInternalModule.attr("XLinkIn").attr("Properties") = xlinkInProperties;
}
