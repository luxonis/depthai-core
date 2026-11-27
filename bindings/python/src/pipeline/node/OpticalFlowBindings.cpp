#include "Common.hpp"
#include "NodeBindings.hpp"
#include "depthai/pipeline/node/OpticalFlow.hpp"
void bind_opticalflow(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::node;
    py::class_<OpticalFlowProperties> properties(m, "OpticalFlowProperties", DOC(dai, OpticalFlowProperties));
    auto opticalFlow = ADD_NODE(OpticalFlow);
    auto* callstack = static_cast<Callstack*>(pCallstack);
    auto callback = callstack->top();
    callstack->pop();
    callback(m, pCallstack);
    properties.def_readonly("reserved", &OpticalFlowProperties::reserved, DOC(dai, OpticalFlowProperties, reserved));
    opticalFlow.def_readonly("inputImage", &OpticalFlow::inputImage, DOC(dai, node, OpticalFlow, inputImage))
        .def_readonly("outputFlow", &OpticalFlow::outputFlow, DOC(dai, node, OpticalFlow, outputFlow))
        .def_readonly("passthroughInputImage", &OpticalFlow::passthroughInputImage, DOC(dai, node, OpticalFlow, passthroughInputImage));
    daiNodeModule.attr("OpticalFlow").attr("Properties") = properties;
}
