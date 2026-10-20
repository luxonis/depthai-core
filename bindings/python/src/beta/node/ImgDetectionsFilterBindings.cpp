
#include "DatatypeBindings.hpp"
#include "depthai/beta/node/ImgDetectionsFilter.hpp"
#include "pipeline/node/Common.hpp"

void bind_beta_imgdetectionsfilter(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::beta::node;

    auto betaModule = m.def_submodule("beta", "Experimental APIs");
    py::class_<beta::ImgDetectionsFilterConfig, Py<beta::ImgDetectionsFilterConfig>, Buffer, std::shared_ptr<beta::ImgDetectionsFilterConfig>>
        imgDetectionsFilterConfig(betaModule, "ImgDetectionsFilterConfig");
    py::class_<beta::ImgDetectionsFilterProperties> imgDetectionsFilterProperties(betaModule, "ImgDetectionsFilterProperties");
    auto imgDetectionsFilter = ADD_BETA_NODE_DERIVED(ImgDetectionsFilter, dai::DeviceNode);

    ///////////////////////////////////////////////////////////////////////
    // Callstack handling
    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);
    ///////////////////////////////////////////////////////////////////////

    imgDetectionsFilterConfig.def(py::init<>())
        .def("__repr__", &beta::ImgDetectionsFilterConfig::str)
        .def_readwrite("labelsToKeep", &beta::ImgDetectionsFilterConfig::labelsToKeep)
        .def_readwrite("labelsToReject", &beta::ImgDetectionsFilterConfig::labelsToReject)
        .def_readwrite("confidenceThreshold", &beta::ImgDetectionsFilterConfig::confidenceThreshold)
        .def_readwrite("minArea", &beta::ImgDetectionsFilterConfig::minArea)
        .def_readwrite("nmsDisabled", &beta::ImgDetectionsFilterConfig::nmsDisabled)
        .def_readwrite("nmsConfidenceThreshold", &beta::ImgDetectionsFilterConfig::nmsConfidenceThreshold)
        .def_readwrite("nmsIouThreshold", &beta::ImgDetectionsFilterConfig::nmsIouThreshold)
        .def_readwrite("sortingDisabled", &beta::ImgDetectionsFilterConfig::sortingDisabled)
        .def_readwrite("sortDescending", &beta::ImgDetectionsFilterConfig::sortDescending)
        .def_readwrite("firstK", &beta::ImgDetectionsFilterConfig::firstK)
        .def("isNoOp", &beta::ImgDetectionsFilterConfig::isNoOp);

    imgDetectionsFilterProperties.def_readwrite("initialConfig", &beta::ImgDetectionsFilterProperties::initialConfig);

    imgDetectionsFilter.def_readonly("input", &ImgDetectionsFilter::input, DOC(dai, beta, node, ImgDetectionsFilter, input))
        .def_readonly("inputConfig", &ImgDetectionsFilter::inputConfig, DOC(dai, beta, node, ImgDetectionsFilter, inputConfig))
        .def_readonly("output", &ImgDetectionsFilter::output, DOC(dai, beta, node, ImgDetectionsFilter, output))
        .def_readonly("initialConfig", &ImgDetectionsFilter::initialConfig, DOC(dai, beta, node, ImgDetectionsFilter, initialConfig))
        .def("setRunOnHost", &ImgDetectionsFilter::setRunOnHost, py::arg("runOnHost"), DOC(dai, beta, node, ImgDetectionsFilter, setRunOnHost))
        .def("runOnHost", &ImgDetectionsFilter::runOnHost, DOC(dai, beta, node, ImgDetectionsFilter, runOnHost));

    imgDetectionsFilter.attr("Properties") = imgDetectionsFilterProperties;
    imgDetectionsFilter.attr("Config") = imgDetectionsFilterConfig;
}
