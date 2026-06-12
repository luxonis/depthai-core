#include <pybind11/chrono.h>
#include <pybind11/numpy.h>

#include "DatatypeBindings.hpp"
#include "depthai/pipeline/datatype/Transformable.hpp"
#include "pipeline/CommonBindings.hpp"

void bind_transformable(pybind11::module& m, void* pCallstack) {
    using namespace dai;

    py::class_<Transformable, std::shared_ptr<Transformable>> transformable(m, "Transformable", DOC(dai, Transformable));
    py::class_<TransformableBuffer, Py<TransformableBuffer>, Buffer, Transformable, std::shared_ptr<TransformableBuffer>> transformableBuffer(
        m, "TransformableBuffer", DOC(dai, TransformableBuffer));

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

    transformable.def_readwrite("transformation", &Transformable::transformation)
        .def("getTransformation", &Transformable::getTransformation, DOC(dai, Transformable, getTransformation))
        .def("setTransformation", &Transformable::setTransformation, py::arg("transformation"), DOC(dai, Transformable, setTransformation));

    transformableBuffer.def(py::init<>()).def("transformTo", &TransformableBuffer::transformTo, py::arg("target"), DOC(dai, TransformableBuffer, transformTo));
}
