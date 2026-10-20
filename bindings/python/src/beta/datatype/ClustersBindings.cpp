#include <pybind11/chrono.h>
#include <pybind11/stl.h>

#include "DatatypeBindings.hpp"
#include "depthai/beta/datatype/Clusters.hpp"
#include "pipeline/CommonBindings.hpp"

void bind_beta_clusters(pybind11::module& m, void* pCallstack) {
    using namespace dai;

    auto betaModule = m.def_submodule("beta", "Experimental APIs");
    py::class_<beta::Cluster> cluster(betaModule, "Cluster", DOC(dai, beta, Cluster));
    py::class_<beta::Clusters, Py<beta::Clusters>, Buffer, Transformable, std::shared_ptr<beta::Clusters>> clusters(
        betaModule, "Clusters", DOC(dai, beta, Clusters));

    ///////////////////////////////////////////////////////////////////////
    // Callstack handling
    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);
    ///////////////////////////////////////////////////////////////////////

    cluster.def(py::init<>())
        .def_readwrite("label", &beta::Cluster::label, DOC(dai, beta, Cluster, label))
        .def_readwrite("points", &beta::Cluster::points, DOC(dai, beta, Cluster, points));

    clusters.def(py::init<>())
        .def("__repr__", &beta::Clusters::str)
        .def_readwrite("clusters", &beta::Clusters::clusters, DOC(dai, beta, Clusters, clusters))
        .def("transformTo", &beta::Clusters::transformTo, py::arg("target"), DOC(dai, beta, Clusters, transformTo))
        .def("getVisualizationMessage", &beta::Clusters::getVisualizationMessage, DOC(dai, beta, Clusters, getVisualizationMessage));
}
