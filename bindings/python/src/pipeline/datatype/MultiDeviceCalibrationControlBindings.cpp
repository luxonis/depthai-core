#include <pybind11/pybind11.h>

#include <memory>

#include "DatatypeBindings.hpp"
#include "depthai/pipeline/datatype/MultiDeviceCalibrationControl.hpp"

void bind_multi_device_calibration_control(pybind11::module& m, void* pCallstack) {
    namespace py = pybind11;
    using namespace dai;
    using Control = MultiDeviceCalibrationControl;
    using Commands = Control::Commands;

    auto control = py::class_<Control, Buffer, std::shared_ptr<Control>>(m, "MultiDeviceCalibrationControl");
    auto commands = py::class_<Commands>(control, "Commands");
    py::class_<Commands::Start>(commands, "Start").def(py::init<>());
    py::class_<Commands::Stop>(commands, "Stop").def(py::init<>());
    py::class_<Commands::Reset>(commands, "Reset").def(py::init<>());

    control.def(py::init<>())
        .def(py::init<const Commands::Start&>())
        .def(py::init<const Commands::Stop&>())
        .def(py::init<const Commands::Reset&>())
        .def_static("start", &Control::start)
        .def_static("stop", &Control::stop)
        .def_static("reset", &Control::reset);

    auto* callstack = static_cast<Callstack*>(pCallstack);
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);
}
