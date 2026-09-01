#include <pybind11/stl.h>

#include "Common.hpp"
#include "NodeBindings.hpp"
#include "depthai/pipeline/node/host/MultiDeviceCalibration.hpp"

void bind_multi_device_calibration(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::node;
    namespace py = pybind11;
    using namespace pybind11::literals;

    auto node = ADD_NODE_DERIVED(MultiDeviceCalibration, ThreadedHostNode);

    auto* callstack = static_cast<Callstack*>(pCallstack);
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);

    node.def_property_readonly(
            "sync", [](MultiDeviceCalibration& self) -> Sync& { return *self.sync; }, py::return_value_policy::reference_internal)
        .def_property_readonly(
            "inputs", [](MultiDeviceCalibration& self) -> Node::InputMap& { return self.inputs; }, py::return_value_policy::reference_internal)
        .def_readonly("inputControl", &MultiDeviceCalibration::inputControl)
        .def_readonly("syncInput", &MultiDeviceCalibration::syncInput)
        .def_readonly("calibrationOutput", &MultiDeviceCalibration::calibrationOutput)
        .def("addCamera", &MultiDeviceCalibration::addCamera, "deviceId"_a, "socket"_a, "cameraOutput"_a)
        .def("setSampleCount", &MultiDeviceCalibration::setSampleCount, "sampleCount"_a)
        .def("getSampleCount", &MultiDeviceCalibration::getSampleCount)
        .def("setKnownDistance",
             &MultiDeviceCalibration::setKnownDistance,
             "fromDeviceId"_a,
             "fromSocket"_a,
             "toDeviceId"_a,
             "toSocket"_a,
             "distance"_a,
             "unit"_a = LengthUnit::CENTIMETER)
        .def("setInitialGuess", &MultiDeviceCalibration::setInitialGuess, "fromDeviceId"_a, "fromSocket"_a, "toDeviceId"_a, "toSocket"_a, "guess"_a)
        .def("setStereoPair", &MultiDeviceCalibration::setStereoPair, "deviceId"_a, "leftSocket"_a, "rightSocket"_a)
        .def("setDeviceCalibration", &MultiDeviceCalibration::setDeviceCalibration, "deviceId"_a, "calibration"_a);
}
