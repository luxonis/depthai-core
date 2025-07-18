#include <memory>
#include <unordered_map>

#include "DatatypeBindings.hpp"
#include "pipeline/CommonBindings.hpp"

// depthai
#include "depthai/pipeline/datatype/EdgeDetectorConfig.hpp"

// pybind
#include <pybind11/chrono.h>
#include <pybind11/numpy.h>

// #include "spdlog/spdlog.h"

void bind_edgedetectorconfig(pybind11::module& m, void* pCallstack) {
    using namespace dai;

    py::class_<EdgeDetectorConfig::EdgeDetectorConfigData> edgeDetectorConfigData(
        m, "EdgeDetectorConfigData", DOC(dai, EdgeDetectorConfig, EdgeDetectorConfigData));
    // py::class_<RawEdgeDetectorConfig, RawBuffer, std::shared_ptr<RawEdgeDetectorConfig>> rawEdgeDetectorConfig(m, "RawEdgeDetectorConfig", DOC(dai,
    // RawEdgeDetectorConfig));
    py::class_<EdgeDetectorConfig, Py<EdgeDetectorConfig>, Buffer, std::shared_ptr<EdgeDetectorConfig>> edgeDetectorConfig(
        m, "EdgeDetectorConfig", DOC(dai, EdgeDetectorConfig));

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
    edgeDetectorConfigData.def(py::init<>())
        .def_readwrite("sobelFilterHorizontalKernel",
                       &EdgeDetectorConfig::EdgeDetectorConfigData::sobelFilterHorizontalKernel,
                       DOC(dai, EdgeDetectorConfig, EdgeDetectorConfigData, sobelFilterHorizontalKernel))
        .def_readwrite("sobelFilterVerticalKernel",
                       &EdgeDetectorConfig::EdgeDetectorConfigData::sobelFilterVerticalKernel,
                       DOC(dai, EdgeDetectorConfig, EdgeDetectorConfigData, sobelFilterVerticalKernel));

    // rawEdgeDetectorConfig
    //     .def(py::init<>())
    //     .def_readwrite("config", &RawEdgeDetectorConfig::config)
    //     ;

    // Message
    edgeDetectorConfig.def(py::init<>())
        .def("__repr__", &EdgeDetectorConfig::str)
        .def("setSobelFilterKernels",
             &EdgeDetectorConfig::setSobelFilterKernels,
             py::arg("horizontalKernel"),
             py::arg("verticalKernel"),
             DOC(dai, EdgeDetectorConfig, setSobelFilterKernels))
        .def("getConfigData", &EdgeDetectorConfig::getConfigData, DOC(dai, EdgeDetectorConfig, getConfigData))
        // .def("get",         &EdgeDetectorConfig::get, DOC(dai, EdgeDetectorConfig, get))
        // .def("set",         &EdgeDetectorConfig::set, py::arg("config"), DOC(dai, EdgeDetectorConfig, set))
        ;
}
