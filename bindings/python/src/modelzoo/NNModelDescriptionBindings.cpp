#include "NNModelDescriptionBindings.hpp"

// depthai
#include "depthai/modelzoo/NNModelDescription.hpp"

void NNModelDescriptionBindings::bind(pybind11::module& m, void* pCallstack) {
    using namespace dai;

    // Type definitions
    py::class_<NNModelDescription> modelDescription(m, "NNModelDescription", DOC(dai, NNModelDescription));

    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    // Call the rest of the type defines, then perform the actual bindings
    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////

    // Bind DeviceBootloader
    modelDescription
        .def(py::init<>(), DOC(dai, NNModelDescription, NNModelDescription))
        .def_static("fromYamlFile", &NNModelDescription::fromYamlFile, py::arg("yamlPath"), DOC(dai, NNModelDescription, fromYamlFile))
        .def("saveToYamlFile", &NNModelDescription::saveToYamlFile, py::arg("yamlPath"), DOC(dai, NNModelDescription, saveToYamlFile))
        .def("toString", &NNModelDescription::toString, DOC(dai, NNModelDescription, toString))
        .def("__str__", &NNModelDescription::toString, DOC(dai, NNModelDescription, toString))
        .def_readwrite("modelSlug", &NNModelDescription::modelSlug, DOC(dai, NNModelDescription, modelSlug))
        .def_readwrite("platform", &NNModelDescription::platform, DOC(dai, NNModelDescription, platform))
        .def_readwrite("modelVersionSlug", &NNModelDescription::modelVersionSlug, DOC(dai, NNModelDescription, modelVersionSlug))
        ;
}