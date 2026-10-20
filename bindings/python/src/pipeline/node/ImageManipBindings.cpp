#include "Common.hpp"
#include "NodeBindings.hpp"
#include "depthai/pipeline/Node.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/ImageManip.hpp"

void bind_imagemanip(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::node;

    // Node and Properties declare upfront
    py::class_<ImageManipProperties, std::shared_ptr<ImageManipProperties>> imageManipProperties(m, "ImageManipProperties", DOC(dai, ImageManipProperties));
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

    // Properties
    py::enum_<ImageManipProperties::PerformanceMode> perfMode(imageManipProperties, "PerformanceMode");
    perfMode.value("BALANCED", ImageManipProperties::PerformanceMode::BALANCED)
        .value("PERFORMANCE", ImageManipProperties::PerformanceMode::PERFORMANCE)
        .value("LOW_POWER", ImageManipProperties::PerformanceMode::LOW_POWER);

    py::enum_<ImageManipProperties::Backend> backend(imageManipProperties, "Backend");
    backend.value("HW", ImageManipProperties::Backend::HW)
        .value("CPU", ImageManipProperties::Backend::CPU)
        .value("GPU", ImageManipProperties::Backend::GPU)
        .value("AUTO", ImageManipProperties::Backend::AUTO);

    imageManipProperties.def_readwrite("initialConfig", &ImageManipProperties::initialConfig)
        .def_readwrite("outputFrameSize", &ImageManipProperties::outputFrameSize)
        .def_readwrite("numFramesPool", &ImageManipProperties::numFramesPool)
        .def_readwrite("maxPoolSize", &ImageManipProperties::maxPoolSize)
        .def_readwrite("backend", &ImageManipProperties::backend)
        .def_readwrite("performanceMode", &ImageManipProperties::performanceMode);

    imageManip.attr("PerformanceMode") = perfMode;
    imageManip.attr("Backend") = backend;

    // ImageManip Node
    imageManip.def_readonly("inputConfig", &ImageManip::inputConfig, DOC(dai, node, ImageManip, inputConfig))
        .def_readonly("inputImage", &ImageManip::inputImage, DOC(dai, node, ImageManip, inputImage))
        .def_readonly("out", &ImageManip::out, DOC(dai, node, ImageManip, out))
        .def_readonly("initialConfig", &ImageManip::initialConfig, DOC(dai, node, ImageManip, initialConfig))
        .def("setRunOnHost", &ImageManip::setRunOnHost, DOC(dai, node, ImageManip, setRunOnHost))
        .def("setBackend", &ImageManip::setBackend, DOC(dai, node, ImageManip, setBackend))
        .def("setPerformanceMode", &ImageManip::setPerformanceMode, DOC(dai, node, ImageManip, setPerformanceMode))
        .def("setNumFramesPool", &ImageManip::setNumFramesPool, DOC(dai, node, ImageManip, setNumFramesPool))
        .def("setMaxOutputFrameSize", &ImageManip::setMaxOutputFrameSize, DOC(dai, node, ImageManip, setMaxOutputFrameSize))
        .def("setMaxPoolSize", &ImageManip::setMaxPoolSize, DOC(dai, node, ImageManip, setMaxPoolSize));

    // Properties alias
    daiNodeModule.attr("ImageManip").attr("Properties") = imageManipProperties;
}
