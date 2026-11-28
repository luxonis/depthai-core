#include <memory>
#include <unordered_map>

#include "DatatypeBindings.hpp"
#include "pipeline/CommonBindings.hpp"

#include "depthai/pipeline/datatype/EdgeDetectorConfig.hpp"

#include <pybind11/chrono.h>
#include <pybind11/numpy.h>

void bind_edgedetectorconfig(pybind11::module& m, void* pCallstack) {
    using namespace dai;

    py::class_<EdgeDetectorConfig::EdgeDetectorConfigData> edgeDetectorConfigData(
        m, "EdgeDetectorConfigData", DOC(dai, EdgeDetectorConfig, EdgeDetectorConfigData));
    py::class_<EdgeDetectorConfig, Py<EdgeDetectorConfig>, Buffer, py::smart_holder> edgeDetectorConfig(m, "EdgeDetectorConfig", DOC(dai, EdgeDetectorConfig));

    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);

    edgeDetectorConfigData.def(py::init<>())
        .def_readwrite("sobelFilterHorizontalKernel",
                       &EdgeDetectorConfig::EdgeDetectorConfigData::sobelFilterHorizontalKernel,
                       DOC(dai, EdgeDetectorConfig, EdgeDetectorConfigData, sobelFilterHorizontalKernel))
        .def_readwrite("sobelFilterVerticalKernel",
                       &EdgeDetectorConfig::EdgeDetectorConfigData::sobelFilterVerticalKernel,
                       DOC(dai, EdgeDetectorConfig, EdgeDetectorConfigData, sobelFilterVerticalKernel));

    edgeDetectorConfig.def(py::init<>())
        .def("__repr__", &EdgeDetectorConfig::str)
        .def("setSobelFilterKernels",
             &EdgeDetectorConfig::setSobelFilterKernels,
             py::arg("horizontalKernel"),
             py::arg("verticalKernel"),
             DOC(dai, EdgeDetectorConfig, setSobelFilterKernels))
        .def("getConfigData", &EdgeDetectorConfig::getConfigData, DOC(dai, EdgeDetectorConfig, getConfigData));
}
