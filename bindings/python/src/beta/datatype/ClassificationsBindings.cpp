#include <pybind11/chrono.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

#include <cstring>

#include "DatatypeBindings.hpp"
#include "depthai/beta/datatype/Classifications.hpp"
#include "pipeline/CommonBindings.hpp"

namespace {

py::array_t<float> toNumpyScores(const std::vector<float>& scores) {
    py::array_t<float> arr(static_cast<py::ssize_t>(scores.size()));
    if(!scores.empty()) {
        std::memcpy(arr.mutable_data(), scores.data(), scores.size() * sizeof(float));
    }
    return arr;
}

}  // namespace

void bind_beta_classifications(pybind11::module& m, void* pCallstack) {
    using namespace dai;

    auto betaModule = m.def_submodule("beta", "Experimental APIs");
    py::class_<beta::Classifications, Py<beta::Classifications>, Buffer, Transformable, std::shared_ptr<beta::Classifications>> classifications(
        betaModule, "Classifications", DOC(dai, beta, Classifications));

    ///////////////////////////////////////////////////////////////////////
    // Callstack handling
    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);
    ///////////////////////////////////////////////////////////////////////

    classifications.def(py::init<>())
        .def("__repr__", &beta::Classifications::str)
        .def_readwrite("classes", &beta::Classifications::classes, DOC(dai, beta, Classifications, classes))
        .def_property(
            "scores",
            [](const beta::Classifications& message) { return toNumpyScores(message.scores); },
            [](beta::Classifications& message, std::vector<float> scores) { message.scores = std::move(scores); },
            DOC(dai, beta, Classifications, scores))
        .def("getTopClass", &beta::Classifications::getTopClass, DOC(dai, beta, Classifications, getTopClass))
        .def("getTopScore", &beta::Classifications::getTopScore, DOC(dai, beta, Classifications, getTopScore))
        .def("transformTo", &beta::Classifications::transformTo, py::arg("target"), DOC(dai, beta, Classifications, transformTo))
        .def("getVisualizationMessage", &beta::Classifications::getVisualizationMessage, DOC(dai, beta, Classifications, getVisualizationMessage));
}
