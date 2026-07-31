#include <pybind11/stl.h>

#include "Common.hpp"
#include "NodeBindings.hpp"
#include "depthai/pipeline/Node.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/host/MultiDeviceCalibration.hpp"

void bind_multi_device_calibration(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::node;
    namespace py = pybind11;
    using namespace pybind11::literals;

    auto multiDeviceCalibration = ADD_NODE_DERIVED(MultiDeviceCalibration, ThreadedHostNode);

    ///////////////////////////////////////////////////////////////////////
    // Call the rest of the type defines, then perform the actual bindings
    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);

    // Actual bindings
    multiDeviceCalibration
        .def_property_readonly(
            "sync", [](MultiDeviceCalibration& node) { return &(*node.sync); }, py::return_value_policy::reference_internal)
        .def_property_readonly(
            "inputs", [](MultiDeviceCalibration& node) { return &node.inputs; }, py::return_value_policy::reference_internal)
        .def_readonly("rigCalibration", &MultiDeviceCalibration::rigCalibration, DOC(dai, node, MultiDeviceCalibration, rigCalibration))
        .def("build", &MultiDeviceCalibration::build, py::arg("sources"), DOC(dai, node, MultiDeviceCalibration, build))
        .def("addCamera", &MultiDeviceCalibration::addCamera, py::arg("frame"), py::arg("source"), DOC(dai, node, MultiDeviceCalibration, addCamera))
        .def("setInitialGuess",
             &MultiDeviceCalibration::setInitialGuess,
             py::arg("from"),
             py::arg("to"),
             py::arg("guess"),
             DOC(dai, node, MultiDeviceCalibration, setInitialGuess))
        .def("setKnownDistance",
             &MultiDeviceCalibration::setKnownDistance,
             py::arg("from"),
             py::arg("to"),
             py::arg("distance"),
             py::arg("unit") = LengthUnit::CENTIMETER,
             DOC(dai, node, MultiDeviceCalibration, setKnownDistance))
        .def("setDeviceCalibration",
             &MultiDeviceCalibration::setDeviceCalibration,
             py::arg("deviceId"),
             py::arg("calibration"),
             DOC(dai, node, MultiDeviceCalibration, setDeviceCalibration))
        .def("setSampleCount", &MultiDeviceCalibration::setSampleCount, py::arg("sampleCount"), DOC(dai, node, MultiDeviceCalibration, setSampleCount))
        .def("setContinuous", &MultiDeviceCalibration::setContinuous, py::arg("continuous"), DOC(dai, node, MultiDeviceCalibration, setContinuous))
        .def("setPerformanceMode", &MultiDeviceCalibration::setPerformanceMode, py::arg("mode"), DOC(dai, node, MultiDeviceCalibration, setPerformanceMode))
        .def("setEstimateInterDeviceScale",
             &MultiDeviceCalibration::setEstimateInterDeviceScale,
             py::arg("enable"),
             DOC(dai, node, MultiDeviceCalibration, setEstimateInterDeviceScale));
}
