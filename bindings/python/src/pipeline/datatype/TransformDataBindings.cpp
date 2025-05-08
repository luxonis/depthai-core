#include <memory>
#include <unordered_map>

#include "DatatypeBindings.hpp"
#include "pipeline/CommonBindings.hpp"

// depthai
#include "depthai/pipeline/datatype/TransformData.hpp"

// pybind
#include <pybind11/chrono.h>
#include <pybind11/numpy.h>

void bind_transformdata(pybind11::module& m, void* pCallstack) {
    using namespace dai;

    py::class_<TransformData, Py<TransformData>, Buffer, std::shared_ptr<TransformData>> transformData(m, "TransformData", DOC(dai, TransformData));

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

    // Metadata / raw
    transformData.def(py::init<>())
        .def(py::init<double, double, double, double, double, double, double, std::string, std::string>(),
             py::arg("x"),
             py::arg("y"),
             py::arg("z"),
             py::arg("qx"),
             py::arg("qy"),
             py::arg("qz"),
             py::arg("qw"),
             py::arg("frameID"),
             py::arg("parentFrameID"))
        .def("__repr__", &TransformData::str)
        .def_readwrite("frameID", &TransformData::frameID)
        .def_readwrite("parentFrameID", &TransformData::parentFrameID)
        .def("getTranslation", &TransformData::getTranslation, DOC(dai, TransformData, getTranslation))
        .def("getRotationEuler", &TransformData::getRotationEuler, DOC(dai, TransformData, getRotationEuler))
        .def("getQuaternion", &TransformData::getQuaternion, DOC(dai, TransformData, getQuaternion));
}
