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

    py::class_<Transform> transform(m, "Transform", DOC(dai, Transform));
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
    transform.def(py::init<>()).def_readwrite("matrix", &Transform::matrix, DOC(dai, Transform, matrix));

    transformData.def(py::init<>(), DOC(dai, TransformData, TransformData))
        .def(py::init<const Transform&>(), py::arg("transform"), DOC(dai, TransformData, TransformData, 2))
        .def(py::init<const std::array<std::array<double, 4>, 4>&>(), py::arg("data"), DOC(dai, TransformData, TransformData, 3))
        .def(py::init<double, double, double, double, double, double, double>(),
             py::arg("x"),
             py::arg("y"),
             py::arg("z"),
             py::arg("qx"),
             py::arg("qy"),
             py::arg("qz"),
             py::arg("qw"),
             DOC(dai, TransformData, TransformData, 4))
        .def(py::init<double, double, double, double, double, double>(),
             py::arg("x"),
             py::arg("y"),
             py::arg("z"),
             py::arg("roll"),
             py::arg("pitch"),
             py::arg("yaw"),
             DOC(dai, TransformData, TransformData, 5))
        .def_readwrite("transform", &TransformData::transform, DOC(dai, TransformData, transform))
        .def("__repr__", &TransformData::str)
        .def("getTranslation", &TransformData::getTranslation, DOC(dai, TransformData, getTranslation))
        .def("getRotationEuler", &TransformData::getRotationEuler, DOC(dai, TransformData, getRotationEuler))
        .def("getQuaternion", &TransformData::getQuaternion, DOC(dai, TransformData, getQuaternion));
}
