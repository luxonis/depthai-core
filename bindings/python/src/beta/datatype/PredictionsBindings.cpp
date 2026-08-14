#include <pybind11/chrono.h>
#include <pybind11/stl.h>

#include "DatatypeBindings.hpp"
#include "depthai/beta/datatype/Predictions.hpp"
#include "pipeline/CommonBindings.hpp"

void bind_beta_predictions(pybind11::module& m, void* pCallstack) {
    using namespace dai;

    auto betaModule = m.def_submodule("beta", "Experimental APIs");
    py::class_<beta::Prediction> prediction(betaModule, "Prediction", DOC(dai, beta, Prediction));
    py::class_<beta::Predictions, Py<beta::Predictions>, Buffer, Transformable, std::shared_ptr<beta::Predictions>> predictions(
        betaModule, "Predictions", DOC(dai, beta, Predictions));

    ///////////////////////////////////////////////////////////////////////
    // Callstack handling
    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);
    ///////////////////////////////////////////////////////////////////////

    prediction.def(py::init<>()).def_readwrite("prediction", &beta::Prediction::prediction, DOC(dai, beta, Prediction, prediction));

    predictions.def(py::init<>())
        .def("__repr__", &beta::Predictions::str)
        .def_readwrite("predictions", &beta::Predictions::predictions, DOC(dai, beta, Predictions, predictions))
        .def("getFirstPrediction", &beta::Predictions::getFirstPrediction, DOC(dai, beta, Predictions, getFirstPrediction))
        .def("transformTo", &beta::Predictions::transformTo, py::arg("target"), DOC(dai, beta, Predictions, transformTo))
        .def("getVisualizationMessage", &beta::Predictions::getVisualizationMessage, DOC(dai, beta, Predictions, getVisualizationMessage));
}
