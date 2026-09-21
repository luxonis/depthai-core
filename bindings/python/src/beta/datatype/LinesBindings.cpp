#include <pybind11/chrono.h>
#include <pybind11/stl.h>

#include "DatatypeBindings.hpp"
#include "depthai/beta/datatype/Lines.hpp"
#include "pipeline/CommonBindings.hpp"

void bind_beta_lines(pybind11::module& m, void* pCallstack) {
    using namespace dai;

    auto betaModule = m.def_submodule("beta", "Experimental APIs");
    py::class_<beta::Line> line(betaModule, "Line", DOC(dai, beta, Line));
    py::class_<beta::Lines, Py<beta::Lines>, Buffer, Transformable, std::shared_ptr<beta::Lines>> lines(betaModule, "Lines", DOC(dai, beta, Lines));

    ///////////////////////////////////////////////////////////////////////
    // Callstack handling
    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);
    ///////////////////////////////////////////////////////////////////////

    line.def(py::init<>())
        .def_readwrite("startPoint", &beta::Line::startPoint, DOC(dai, beta, Line, startPoint))
        .def_readwrite("endPoint", &beta::Line::endPoint, DOC(dai, beta, Line, endPoint))
        .def_readwrite("confidence", &beta::Line::confidence, DOC(dai, beta, Line, confidence));

    lines.def(py::init<>())
        .def("__repr__", &beta::Lines::str)
        .def_readwrite("lines", &beta::Lines::lines, DOC(dai, beta, Lines, lines))
        .def("transformTo", &beta::Lines::transformTo, py::arg("target"), DOC(dai, beta, Lines, transformTo))
        .def("getVisualizationMessage", &beta::Lines::getVisualizationMessage, DOC(dai, beta, Lines, getVisualizationMessage));
}
