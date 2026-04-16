#include <memory>

#include "DatatypeBindings.hpp"
#include "pipeline/CommonBindings.hpp"

// depthai
#include "depthai/pipeline/datatype/PointCloudConfig.hpp"

// pybind
#include <pybind11/chrono.h>
#include <pybind11/numpy.h>

// #include "spdlog/spdlog.h"

void bind_pointcloudconfig(pybind11::module& m, void* pCallstack) {
    using namespace dai;

    // py::class_<RawPointCloudConfig, RawBuffer, std::shared_ptr<RawPointCloudConfig>> rawConfig(m, "RawPointCloudConfig", DOC(dai, RawPointCloudConfig));
    py::class_<PointCloudConfig, Py<PointCloudConfig>, Buffer, std::shared_ptr<PointCloudConfig>> config(m, "PointCloudConfig", DOC(dai, PointCloudConfig));

    py::enum_<PointCloudConfig::CoordinateSystemType>(config, "CoordinateSystemType")
        .value("DEFAULT", PointCloudConfig::CoordinateSystemType::DEFAULT)
        .value("CAMERA_SOCKET", PointCloudConfig::CoordinateSystemType::CAMERA_SOCKET)
        .value("HOUSING", PointCloudConfig::CoordinateSystemType::HOUSING);

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

    // // Metadata / raw
    // rawConfig
    //     .def(py::init<>())
    //     .def_readwrite("sparse", &RawPointCloudConfig::sparse, DOC(dai, RawPointCloudConfig, sparse))
    //     .def_readwrite("transformationMatrix", &RawPointCloudConfig::transformationMatrix, DOC(dai, RawPointCloudConfig, transformationMatrix))
    //     ;

    // Message
    config.def(py::init<>())
        .def("__repr__", &PointCloudConfig::str)
        .def("getOrganized", &PointCloudConfig::getOrganized, DOC(dai, PointCloudConfig, getOrganized))
        .def("setOrganized", &PointCloudConfig::setOrganized, DOC(dai, PointCloudConfig, setOrganized))
        .def("getTransformationMatrix", &PointCloudConfig::getTransformationMatrix, DOC(dai, PointCloudConfig, getTransformationMatrix))
        .def(
            "setTransformationMatrix",
            [](PointCloudConfig& cfg, std::array<std::array<float, 3>, 3> mat) { return cfg.setTransformationMatrix(mat); },
            DOC(dai, PointCloudConfig, setTransformationMatrix))
        .def(
            "setTransformationMatrix",
            [](PointCloudConfig& cfg, std::array<std::array<float, 4>, 4> mat) { return cfg.setTransformationMatrix(mat); },
            DOC(dai, PointCloudConfig, setTransformationMatrix))
        .def("getLengthUnit", &PointCloudConfig::getLengthUnit, DOC(dai, PointCloudConfig, getLengthUnit))
        .def("setLengthUnit", &PointCloudConfig::setLengthUnit, DOC(dai, PointCloudConfig, setLengthUnit))
        .def("setTargetCoordinateSystem",
             py::overload_cast<CameraBoardSocket, bool>(&PointCloudConfig::setTargetCoordinateSystem),
             py::arg("targetCamera"),
             py::arg("useSpecTranslation") = false,
             DOC(dai, PointCloudConfig, setTargetCoordinateSystem))
        .def("setTargetCoordinateSystem",
             py::overload_cast<HousingCoordinateSystem, bool>(&PointCloudConfig::setTargetCoordinateSystem),
             py::arg("housingCS"),
             py::arg("useSpecTranslation") = true,
             DOC(dai, PointCloudConfig, setTargetCoordinateSystem, 2))
        .def("getCoordinateSystemType", &PointCloudConfig::getCoordinateSystemType, DOC(dai, PointCloudConfig, getCoordinateSystemType))
        .def("getTargetCameraSocket", &PointCloudConfig::getTargetCameraSocket, DOC(dai, PointCloudConfig, getTargetCameraSocket))
        .def("getTargetHousingCS", &PointCloudConfig::getTargetHousingCS, DOC(dai, PointCloudConfig, getTargetHousingCS))
        .def("getUseSpecTranslation", &PointCloudConfig::getUseSpecTranslation, DOC(dai, PointCloudConfig, getUseSpecTranslation))
        // Deprecated
        .def(
            "getSparse",
            [](const PointCloudConfig& cfg) {
                if(PyErr_WarnEx(PyExc_DeprecationWarning, "getSparse() is deprecated, use getOrganized() instead (note: sparse == !organized).", 1) < 0)
                    throw py::error_already_set();
                return !cfg.getOrganized();
            },
            "**Deprecated:** Use getOrganized() instead (sparse == !organized).")
        .def(
            "setSparse",
            [](PointCloudConfig& cfg, bool sparse) -> PointCloudConfig& {
                if(PyErr_WarnEx(PyExc_DeprecationWarning, "setSparse() is deprecated, use setOrganized() instead (note: sparse == !organized).", 1) < 0)
                    throw py::error_already_set();
                return cfg.setOrganized(!sparse);
            },
            py::arg("sparse"),
            "**Deprecated:** Use setOrganized() instead (sparse == !organized).");

    // add aliases
}
