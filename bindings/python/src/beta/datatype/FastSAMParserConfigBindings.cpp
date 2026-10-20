#include "DatatypeBindings.hpp"
#include "depthai/beta/datatype/FastSAMParserConfig.hpp"
#include "pipeline/CommonBindings.hpp"

void bind_beta_fastsamparserconfig(pybind11::module& m, void* pCallstack) {
    using namespace dai;

    auto betaModule = m.def_submodule("beta", "Experimental APIs");
    py::class_<beta::FastSAMParserConfig, Py<beta::FastSAMParserConfig>, Buffer, std::shared_ptr<beta::FastSAMParserConfig>> config(
        betaModule, "FastSAMParserConfig", DOC(dai, beta, FastSAMParserConfig));
    py::enum_<beta::FastSAMParserConfig::Prompt> prompt(config, "Prompt", DOC(dai, beta, FastSAMParserConfig, Prompt));

    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);

    prompt.value("EVERYTHING", beta::FastSAMParserConfig::Prompt::EVERYTHING, DOC(dai, beta, FastSAMParserConfig, Prompt, EVERYTHING))
        .value("POINT", beta::FastSAMParserConfig::Prompt::POINT, DOC(dai, beta, FastSAMParserConfig, Prompt, POINT))
        .value("BOUNDING_BOX", beta::FastSAMParserConfig::Prompt::BOUNDING_BOX, DOC(dai, beta, FastSAMParserConfig, Prompt, BOUNDING_BOX));

    config.def(py::init<>())
        .def("__repr__", &beta::FastSAMParserConfig::str)
        .def_readwrite("confidenceThreshold", &beta::FastSAMParserConfig::confidenceThreshold)
        .def_readwrite("iouThreshold", &beta::FastSAMParserConfig::iouThreshold)
        .def_readwrite("maskConfidence", &beta::FastSAMParserConfig::maskConfidence)
        .def_readwrite("prompt", &beta::FastSAMParserConfig::prompt)
        .def_readwrite("points", &beta::FastSAMParserConfig::points)
        .def_readwrite("pointLabel", &beta::FastSAMParserConfig::pointLabel)
        .def_readwrite("boundingBox", &beta::FastSAMParserConfig::boundingBox)
        .def("setConfidenceThreshold",
             &beta::FastSAMParserConfig::setConfidenceThreshold,
             py::arg("threshold"),
             DOC(dai, beta, FastSAMParserConfig, setConfidenceThreshold))
        .def("getConfidenceThreshold", &beta::FastSAMParserConfig::getConfidenceThreshold, DOC(dai, beta, FastSAMParserConfig, getConfidenceThreshold))
        .def("setIouThreshold", &beta::FastSAMParserConfig::setIouThreshold, py::arg("threshold"), DOC(dai, beta, FastSAMParserConfig, setIouThreshold))
        .def("getIouThreshold", &beta::FastSAMParserConfig::getIouThreshold, DOC(dai, beta, FastSAMParserConfig, getIouThreshold))
        .def("setMaskConfidence", &beta::FastSAMParserConfig::setMaskConfidence, py::arg("threshold"), DOC(dai, beta, FastSAMParserConfig, setMaskConfidence))
        .def("getMaskConfidence", &beta::FastSAMParserConfig::getMaskConfidence, DOC(dai, beta, FastSAMParserConfig, getMaskConfidence))
        .def("setPrompt", &beta::FastSAMParserConfig::setPrompt, py::arg("prompt"), DOC(dai, beta, FastSAMParserConfig, setPrompt))
        .def("getPrompt", &beta::FastSAMParserConfig::getPrompt, DOC(dai, beta, FastSAMParserConfig, getPrompt))
        .def("setPoints", &beta::FastSAMParserConfig::setPoints, py::arg("x"), py::arg("y"), DOC(dai, beta, FastSAMParserConfig, setPoints))
        .def("getPoints", &beta::FastSAMParserConfig::getPoints, DOC(dai, beta, FastSAMParserConfig, getPoints))
        .def("setPointLabel", &beta::FastSAMParserConfig::setPointLabel, py::arg("label"), DOC(dai, beta, FastSAMParserConfig, setPointLabel))
        .def("getPointLabel", &beta::FastSAMParserConfig::getPointLabel, DOC(dai, beta, FastSAMParserConfig, getPointLabel))
        .def("setBoundingBox", &beta::FastSAMParserConfig::setBoundingBox, py::arg("boundingBox"), DOC(dai, beta, FastSAMParserConfig, setBoundingBox))
        .def("getBoundingBox", &beta::FastSAMParserConfig::getBoundingBox, DOC(dai, beta, FastSAMParserConfig, getBoundingBox))
        .def("validate", &beta::FastSAMParserConfig::validate, DOC(dai, beta, FastSAMParserConfig, validate));
}
