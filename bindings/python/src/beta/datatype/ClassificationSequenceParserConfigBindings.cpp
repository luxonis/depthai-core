#include "DatatypeBindings.hpp"
#include "depthai/beta/datatype/ClassificationSequenceParserConfig.hpp"
#include "pipeline/CommonBindings.hpp"

void bind_beta_classificationsequenceparserconfig(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    auto betaModule = m.def_submodule("beta", "Experimental APIs");
    py::class_<beta::ClassificationSequenceParserConfig,
               Py<beta::ClassificationSequenceParserConfig>,
               Buffer,
               std::shared_ptr<beta::ClassificationSequenceParserConfig>>
        config(betaModule, "ClassificationSequenceParserConfig", DOC(dai, beta, ClassificationSequenceParserConfig));

    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);

    config.def(py::init<>())
        .def("__repr__", &beta::ClassificationSequenceParserConfig::str)
        .def_readwrite("ignoredIndexes", &beta::ClassificationSequenceParserConfig::ignoredIndexes)
        .def_readwrite("removeDuplicates", &beta::ClassificationSequenceParserConfig::removeDuplicates)
        .def_readwrite("concatenateClasses", &beta::ClassificationSequenceParserConfig::concatenateClasses)
        .def("setIgnoredIndexes",
             &beta::ClassificationSequenceParserConfig::setIgnoredIndexes,
             py::arg("indexes"),
             DOC(dai, beta, ClassificationSequenceParserConfig, setIgnoredIndexes))
        .def("getIgnoredIndexes",
             &beta::ClassificationSequenceParserConfig::getIgnoredIndexes,
             DOC(dai, beta, ClassificationSequenceParserConfig, getIgnoredIndexes))
        .def("setRemoveDuplicates",
             &beta::ClassificationSequenceParserConfig::setRemoveDuplicates,
             py::arg("enabled"),
             DOC(dai, beta, ClassificationSequenceParserConfig, setRemoveDuplicates))
        .def("getRemoveDuplicates",
             &beta::ClassificationSequenceParserConfig::getRemoveDuplicates,
             DOC(dai, beta, ClassificationSequenceParserConfig, getRemoveDuplicates))
        .def("setConcatenateClasses",
             &beta::ClassificationSequenceParserConfig::setConcatenateClasses,
             py::arg("enabled"),
             DOC(dai, beta, ClassificationSequenceParserConfig, setConcatenateClasses))
        .def("getConcatenateClasses",
             &beta::ClassificationSequenceParserConfig::getConcatenateClasses,
             DOC(dai, beta, ClassificationSequenceParserConfig, getConcatenateClasses))
        .def("validate", &beta::ClassificationSequenceParserConfig::validate, DOC(dai, beta, ClassificationSequenceParserConfig, validate));
}
