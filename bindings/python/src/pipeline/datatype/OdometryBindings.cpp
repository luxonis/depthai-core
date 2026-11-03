#include <memory>

#include "DatatypeBindings.hpp"
#include "pipeline/CommonBindings.hpp"

// depthai
#include "depthai/pipeline/datatype/Odometry.hpp"

void bind_odometry(pybind11::module& m, void* pCallstack) {
    using namespace dai;

    py::class_<Odometry, Py<Odometry>, TransformData, std::shared_ptr<Odometry>> odometry(m, "Odometry", DOC(dai, Odometry));

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

    odometry.def(py::init<>())
        .def(py::init<double, double, double, double, double, double, double, const Point3d&>(),
             py::arg("x"),
             py::arg("y"),
             py::arg("z"),
             py::arg("qx"),
             py::arg("qy"),
             py::arg("qz"),
             py::arg("qw"),
             py::arg("velocity"),
             DOC(dai, Odometry, Odometry))
        .def("__repr__", &Odometry::str)
        .def_readwrite("velocity", &Odometry::velocity);
}
