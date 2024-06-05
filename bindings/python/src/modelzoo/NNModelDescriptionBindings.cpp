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
    modelDescription.def_static("fromYaml", &NNModelDescription::fromYaml, py::arg("yamlPath"), DOC(dai, NNModelDescription, fromYaml))
        .def_static("fromParameters",
                    &NNModelDescription::fromParameters,
                    py::arg("name"),
                    py::arg("version"),
                    py::arg("platform"),
                    DOC(dai, NNModelDescription, fromParameters))
        .def("getName", &NNModelDescription::getName, DOC(dai, NNModelDescription, getName))
        .def("getVersion", &NNModelDescription::getVersion, DOC(dai, NNModelDescription, getVersion))
        .def("getPlatform", &NNModelDescription::getPlatform, DOC(dai, NNModelDescription, getPlatform))
        .def("setName", &NNModelDescription::setName, py::arg("name"), DOC(dai, NNModelDescription, setName))
        .def("setVersion", &NNModelDescription::setVersion, py::arg("version"), DOC(dai, NNModelDescription, setVersion))
        .def("setPlatform", &NNModelDescription::setPlatform, py::arg("platform"), DOC(dai, NNModelDescription, setPlatform));
}