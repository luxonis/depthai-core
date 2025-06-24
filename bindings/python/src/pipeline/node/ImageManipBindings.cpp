#include "Common.hpp"
#include "NodeBindings.hpp"
#include "depthai/pipeline/Node.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/ImageManip.hpp"

void bind_imagemanip(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::node;

    // Node and Properties declare upfront
    // TODO(themarpe) - properties
    auto imageManip = ADD_NODE(ImageManip);

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
    py::enum_<ImageManip::PerformanceMode> perfMode(imageManip, "PerformanceMode");
    perfMode.value("BALANCED", ImageManip::PerformanceMode::BALANCED)
        .value("PERFORMANCE", ImageManip::PerformanceMode::PERFORMANCE)
        .value("LOW_POWER", ImageManip::PerformanceMode::LOW_POWER);

    py::enum_<ImageManip::Backend> backend(imageManip, "Backend");
    backend.value("HW", ImageManip::Backend::HW).value("CPU", ImageManip::Backend::CPU);

    // ImageManip Node
    imageManip.def_readonly("inputConfig", &ImageManip::inputConfig, DOC(dai, node, ImageManip, inputConfig))
        .def_readonly("inputImage", &ImageManip::inputImage, DOC(dai, node, ImageManip, inputImage))
        .def_readonly("out", &ImageManip::out, DOC(dai, node, ImageManip, out))
        .def_readonly("initialConfig", &ImageManip::initialConfig, DOC(dai, node, ImageManip, initialConfig))
        .def("setRunOnHost", &ImageManip::setRunOnHost, DOC(dai, node, ImageManip, setRunOnHost))
        .def("setBackend", &ImageManip::setBackend, DOC(dai, node, ImageManip, setBackend))
        .def("setPerformanceMode", &ImageManip::setPerformanceMode, DOC(dai, node, ImageManip, setPerformanceMode))
        .def("setNumFramesPool", &ImageManip::setNumFramesPool, DOC(dai, node, ImageManip, setNumFramesPool))
        .def("setMaxOutputFrameSize", &ImageManip::setMaxOutputFrameSize, DOC(dai, node, ImageManip, setMaxOutputFrameSize));
}
