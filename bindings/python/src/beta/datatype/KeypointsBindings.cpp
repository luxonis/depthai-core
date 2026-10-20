#include <pybind11/chrono.h>
#include <pybind11/stl.h>

#include "DatatypeBindings.hpp"
#include "depthai/beta/datatype/Keypoints.hpp"
#include "pipeline/CommonBindings.hpp"

void bind_beta_keypoints(pybind11::module& m, void* pCallstack) {
    using namespace dai;

    auto betaModule = m.def_submodule("beta", "Experimental APIs");
    py::class_<beta::Keypoints, Py<beta::Keypoints>, Buffer, Transformable, std::shared_ptr<beta::Keypoints>> keypoints(
        betaModule, "Keypoints", DOC(dai, beta, Keypoints));

    ///////////////////////////////////////////////////////////////////////
    // Callstack handling
    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);
    ///////////////////////////////////////////////////////////////////////

    keypoints.def(py::init<>())
        .def("__repr__", &beta::Keypoints::str)
        .def_readwrite("keypointsList", &beta::Keypoints::keypointsList, DOC(dai, beta, Keypoints, keypointsList))
        .def("getKeypoints", &beta::Keypoints::getKeypoints, DOC(dai, beta, Keypoints, getKeypoints))
        .def("setKeypoints",
             py::overload_cast<const std::vector<Keypoint>&>(&beta::Keypoints::setKeypoints),
             py::arg("keypoints"),
             DOC(dai, beta, Keypoints, setKeypoints))
        .def("setKeypoints",
             py::overload_cast<const std::vector<Keypoint>&, const std::vector<Edge>&>(&beta::Keypoints::setKeypoints),
             py::arg("keypoints"),
             py::arg("edges"),
             DOC(dai, beta, Keypoints, setKeypoints, 2))
        .def("getEdges", &beta::Keypoints::getEdges, DOC(dai, beta, Keypoints, getEdges))
        .def("setEdges", &beta::Keypoints::setEdges, py::arg("edges"), DOC(dai, beta, Keypoints, setEdges))
        .def("getPoints2f", &beta::Keypoints::getPoints2f, DOC(dai, beta, Keypoints, getPoints2f))
        .def("getPoints3f", &beta::Keypoints::getPoints3f, DOC(dai, beta, Keypoints, getPoints3f))
        .def("transformTo", &beta::Keypoints::transformTo, py::arg("target"), DOC(dai, beta, Keypoints, transformTo))
        .def("getVisualizationMessage", &beta::Keypoints::getVisualizationMessage, DOC(dai, beta, Keypoints, getVisualizationMessage));
}
