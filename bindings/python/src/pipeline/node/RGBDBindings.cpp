#include <pybind11/eval.h>

#include "Common.hpp"
#include "NodeBindings.hpp"
#include "depthai/pipeline/ThreadedHostNode.hpp"
#include "depthai/pipeline/node/host/RGBD.hpp"

extern py::handle daiNodeModule;

void bind_rgbd(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::node;

    // declare upfront
    auto rgbdNode = ADD_NODE_DERIVED(RGBD, ThreadedHostNode);

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

    // RGBD Node
    rgbdNode.def_property_readonly(
                "inColor", [](RGBD& node) { return &node.inColor; }, py::return_value_policy::reference_internal)
        .def_property_readonly(
            "inDepth", [](RGBD& node) { return &node.inDepth; }, py::return_value_policy::reference_internal)
        .def_readonly("pcl", &RGBD::pcl, DOC(dai, node, RGBD, pcl))
        .def_readonly("rgbd", &RGBD::rgbd, DOC(dai, node, RGBD, rgbd))
        .def("build", static_cast<std::shared_ptr<RGBD> (RGBD::*)()>(&RGBD::build))
        .def("build",
             static_cast<std::shared_ptr<RGBD> (RGBD::*)(bool, StereoDepth::PresetMode, std::pair<int, int>, std::optional<float>)>(&RGBD::build),
             py::arg("autocreate"),
             py::arg("mode") = StereoDepth::PresetMode::DEFAULT,
             py::arg("frameSize") = std::make_pair(640, 400),
             py::arg("fps") = std::nullopt,
             DOC(dai, node, RGBD, build, 2))
        // Build method with DepthSource variant
        .def("build",
             py::overload_cast<const std::shared_ptr<Camera>&, const node::DepthSource&, std::pair<int, int>, std::optional<float>>(&RGBD::build),
             py::arg("camera"),
             py::arg("depthSource"),
             py::arg("frameSize") = std::make_pair(640, 400),
             py::arg("fps") = std::nullopt,
             DOC(dai, node, RGBD, build, 3))
        .def("setDepthUnits", &RGBD::setDepthUnit, py::arg("units"), DOC(dai, node, RGBD, setDepthUnit))
        .def("setHousingTransformation", &RGBD::setHousingTransformation, py::arg("housingCS"), py::arg("useSpecTranslation") = true, DOC(dai, node, RGBD, setHousingTransformation))
        .def("useCPU", &RGBD::useCPU, DOC(dai, node, RGBD, useCPU))
        .def("useCPUMT", &RGBD::useCPUMT, py::arg("numThreads") = 2, DOC(dai, node, RGBD, useCPUMT))
        .def("useGPU", &RGBD::useGPU, py::arg("device") = 0, DOC(dai, node, RGBD, useGPU))
        .def("printDevices", &RGBD::printDevices, DOC(dai, node, RGBD, printDevices));
}
