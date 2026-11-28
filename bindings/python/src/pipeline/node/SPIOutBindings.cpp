#include "Common.hpp"
#include "NodeBindings.hpp"
#include "depthai/pipeline/Node.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/SPIOut.hpp"

void bind_spiout(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::node;

    py::class_<SPIOutProperties> spiOutProperties(m, "SPIOutProperties", DOC(dai, SPIOutProperties));
    auto spiOut = ADD_NODE(SPIOut);

    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);

    spiOutProperties.def_readwrite("streamName", &SPIOutProperties::streamName).def_readwrite("busId", &SPIOutProperties::busId);

    spiOut.def_readonly("input", &SPIOut::input, DOC(dai, node, SPIOut, input))
        .def("setStreamName", &SPIOut::setStreamName, py::arg("name"), DOC(dai, node, SPIOut, setStreamName))
        .def("setBusId", &SPIOut::setBusId, py::arg("id"), DOC(dai, node, SPIOut, setBusId));
    daiNodeModule.attr("SPIOut").attr("Properties") = spiOutProperties;
}
