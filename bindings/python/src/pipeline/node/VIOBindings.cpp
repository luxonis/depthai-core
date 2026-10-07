#include "Common.hpp"
#include "NodeBindings.hpp"
#include "depthai/pipeline/node/VIO.hpp"

void bind_vio(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::node;

    py::class_<VIOProperties> properties(m, "VIOProperties", DOC(dai, VIOProperties));
    auto vio = ADD_NODE(VIO);

    Callstack* callstack = static_cast<Callstack*>(pCallstack);
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);

    properties.def_readwrite("imuFrequency", &VIOProperties::imuFrequency, DOC(dai, VIOProperties, imuFrequency))
        .def_readwrite("useSpecTranslation", &VIOProperties::useSpecTranslation, DOC(dai, VIOProperties, useSpecTranslation));
    vio.def_readonly("stereo", &VIO::stereo, DOC(dai, node, VIO, stereo))
        .def_readonly("imu", &VIO::imu, DOC(dai, node, VIO, imu))
        .def_readonly("transform", &VIO::transform, DOC(dai, node, VIO, transform))
        .def("setImuUpdateRate", &VIO::setImuUpdateRate, py::arg("rate"), py::return_value_policy::reference_internal, DOC(dai, node, VIO, setImuUpdateRate))
        .def("setUseSpecTranslation",
             &VIO::setUseSpecTranslation,
             py::arg("use"),
             py::return_value_policy::reference_internal,
             DOC(dai, node, VIO, setUseSpecTranslation));
    daiNodeModule.attr("VIO").attr("Properties") = properties;
}
