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
                    py::overload_cast<const std::string&, const std::string&, const std::string&>(&NNModelDescription::fromParameters),
                    py::arg("modelSlug"),
                    py::arg("platform"),
                    py::arg("modelInstanceSlug"),
                    DOC(dai, NNModelDescription, fromParameters))
        .def_static("fromParameters",
                    py::overload_cast<const std::string&, const Platform, const std::string&>(&NNModelDescription::fromParameters),
                    py::arg("modelSlug"),
                    py::arg("platform"),
                    py::arg("modelInstanceSlug"),
                    DOC(dai, NNModelDescription, fromParameters))
        .def("toString", &NNModelDescription::toString, DOC(dai, NNModelDescription, toString))
        .def("getModelSlug", &NNModelDescription::getModelSlug, DOC(dai, NNModelDescription, getModelSlug))
        .def("getPlatform", &NNModelDescription::getPlatform, DOC(dai, NNModelDescription, getPlatform))
        .def("getModelInstanceSlug", &NNModelDescription::getModelInstanceSlug, DOC(dai, NNModelDescription, getModelInstanceSlug))
        .def("setModelSlug", &NNModelDescription::setModelSlug, py::arg("modelSlug"), DOC(dai, NNModelDescription, setModelSlug))
        .def("setPlatform", &NNModelDescription::setPlatform, py::arg("platform"), DOC(dai, NNModelDescription, setPlatform))
        .def(
            "setModelInstanceSlug", &NNModelDescription::setModelInstanceSlug, py::arg("modelInstanceSlug"), DOC(dai, NNModelDescription, setModelInstanceSlug))
        .def("__str__", &NNModelDescription::toString, DOC(dai, NNModelDescription, toString))
        .def("__repr__", &NNModelDescription::toString, DOC(dai, NNModelDescription, toString));
}