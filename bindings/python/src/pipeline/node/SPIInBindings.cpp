#include "Common.hpp"
#include "NodeBindings.hpp"
#include "depthai/pipeline/Node.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/SPIIn.hpp"

void bind_spiin(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::node;

    py::class_<SPIInProperties> spiInProperties(m, "SPIInProperties", DOC(dai, SPIInProperties));
    auto spiIn = ADD_NODE(SPIIn);

    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);

    spiInProperties.def_readwrite("streamName", &SPIInProperties::streamName)
        .def_readwrite("busId", &SPIInProperties::busId)
        .def_readwrite("maxDataSize", &SPIInProperties::maxDataSize)
        .def_readwrite("numFrames", &SPIInProperties::numFrames);

    spiIn.def_readonly("out", &SPIIn::out, DOC(dai, node, SPIIn, out))
        .def("setStreamName", &SPIIn::setStreamName, py::arg("name"), DOC(dai, node, SPIIn, setStreamName))
        .def("setBusId", &SPIIn::setBusId, py::arg("id"), DOC(dai, node, SPIIn, setBusId))
        .def("setMaxDataSize", &SPIIn::setMaxDataSize, py::arg("maxDataSize"), DOC(dai, node, SPIIn, setMaxDataSize))
        .def("setNumFrames", &SPIIn::setNumFrames, py::arg("numFrames"), DOC(dai, node, SPIIn, setNumFrames))
        .def("getStreamName", &SPIIn::getStreamName, DOC(dai, node, SPIIn, getStreamName))
        .def("getBusId", &SPIIn::getBusId, DOC(dai, node, SPIIn, getBusId))
        .def("getMaxDataSize", &SPIIn::getMaxDataSize, DOC(dai, node, SPIIn, getMaxDataSize))
        .def("getNumFrames", &SPIIn::getNumFrames, DOC(dai, node, SPIIn, getNumFrames));
    daiNodeModule.attr("SPIIn").attr("Properties") = spiInProperties;
}
