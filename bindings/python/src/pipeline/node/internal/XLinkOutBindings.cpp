#include "../Common.hpp"
#include "../NodeBindings.hpp"
#include "depthai/pipeline/Node.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/internal/XLinkOut.hpp"

extern py::handle daiNodeInternalModule;

void bind_xlinkout(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::node;
    using namespace dai::node::internal;

    // Node and Properties declare upfront
    py::class_<XLinkOutProperties> xlinkOutProperties(m, "XLinkOutProperties", DOC(dai, internal, XLinkOutProperties));
    auto xlinkOut = ADD_NODE_INTERNAL(XLinkOut);

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
    xlinkOutProperties.def_readwrite("maxFpsLimit", &XLinkOutProperties::maxFpsLimit)
        .def_readwrite("streamName", &XLinkOutProperties::streamName)
        .def_readwrite("metadataOnly", &XLinkOutProperties::metadataOnly);

    // Node
    xlinkOut.def_readonly("input", &XLinkOut::input, DOC(dai, node, internal, XLinkOut, input))
        .def("setStreamName", &XLinkOut::setStreamName, py::arg("streamName"), DOC(dai, node, internal, XLinkOut, setStreamName))
        .def("setFpsLimit", &XLinkOut::setFpsLimit, py::arg("fpsLimit"), DOC(dai, node, internal, XLinkOut, setFpsLimit))
        .def("getStreamName", &XLinkOut::getStreamName, DOC(dai, node, internal, XLinkOut, getStreamName))
        .def("getFpsLimit", &XLinkOut::getFpsLimit, DOC(dai, node, internal, XLinkOut, getFpsLimit))
        .def("setMetadataOnly", &XLinkOut::setMetadataOnly, DOC(dai, node, internal, XLinkOut, setMetadataOnly))
        .def("getMetadataOnly", &XLinkOut::getMetadataOnly, DOC(dai, node, internal, XLinkOut, getMetadataOnly));
    daiNodeInternalModule.attr("XLinkOut").attr("Properties") = xlinkOutProperties;
}
