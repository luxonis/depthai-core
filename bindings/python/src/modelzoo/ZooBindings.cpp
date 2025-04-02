#include "ZooBindings.hpp"

// depthai
#include "depthai/modelzoo/Zoo.hpp"

void ZooBindings::bind(pybind11::module& m, void* pCallstack) {
    using namespace dai;

    // Type definitions
    py::class_<NNModelDescription> modelDescription(m, "NNModelDescription", DOC(dai, NNModelDescription));
    py::class_<SlugComponents> slugComponents(m, "SlugComponents", DOC(dai, SlugComponents));
    py::implicitly_convertible<std::string, NNModelDescription>();

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

    m.def("getModelFromZoo",
          getModelFromZoo,
          py::arg("modelDescription"),
          py::arg("useCached") = true,
          py::arg("cacheDirectory") = MODEL_ZOO_DEFAULT_CACHE_PATH,
          py::arg("apiKey") = "",
          DOC(dai, getModelFromZoo));

    m.def("downloadModelsFromZoo",
          downloadModelsFromZoo,
          py::arg("path"),
          py::arg("cacheDirectory") = MODEL_ZOO_DEFAULT_CACHE_PATH,
          py::arg("apiKey") = "",
          DOC(dai, downloadModelsFromZoo));

    // Bind NNModelDescription
    modelDescription.def(py::init<>())
        .def(py::init([](const std::string& model,
                         const std::string& platform,
                         const std::string& optimizationLevel,
                         const std::string& compressionLevel,
                         const std::string& snpeVersion,
                         const std::string& modelPrecisionType) {
                 return NNModelDescription{model, platform, optimizationLevel, compressionLevel, snpeVersion, modelPrecisionType};
             }),
             py::arg("model"),
             py::arg("platform") = "",
             py::arg("optimizationLevel") = "",
             py::arg("compressionLevel") = "",
             py::arg("snpeVersion") = "",
             py::arg("modelPrecisionType") = "")

        .def(py::init([](const std::string& modelSlug,
                         const std::string& platform,
                         const std::string& modelVersionSlug,
                         const std::string& modelInstanceHash,
                         const std::string& optimizationLevel,
                         const std::string& compressionLevel,
                         const std::string& teamName,
                         const std::string& snpeVersion,
                         const std::string& modelPrecisionType) {
                 PyErr_WarnEx(PyExc_DeprecationWarning,
                              "Constructor with split out model slug is deprecated, use NNModelDescription(model, platform, optimizationLevel, ...) instead.",
                              1);
                 SlugComponents slugComponents = SlugComponents{teamName, modelSlug, modelVersionSlug, modelInstanceHash};
                 return NNModelDescription{slugComponents.merge(), platform, optimizationLevel, compressionLevel, snpeVersion, modelPrecisionType};
             }),
             py::arg("modelSlug"),
             py::arg("platform") = "",
             py::arg("modelVersionSlug") = "",
             py::arg("modelInstanceHash") = "",
             py::arg("optimizationLevel") = "",
             py::arg("compressionLevel") = "",
             py::arg("teamName") = "",
             py::arg("snpeVersion") = "",
             py::arg("modelPrecisionType") = "")

        .def(py::init<>([](const std::string& model) { return NNModelDescription{model}; }), py::arg("model"))
        .def_static("fromYamlFile", &NNModelDescription::fromYamlFile, py::arg("yamlPath"), DOC(dai, NNModelDescription, fromYamlFile))
        .def("saveToYamlFile", &NNModelDescription::saveToYamlFile, py::arg("yamlPath"), DOC(dai, NNModelDescription, saveToYamlFile))
        .def("check", &NNModelDescription::check, DOC(dai, NNModelDescription, check))
        .def("toString", &NNModelDescription::toString, DOC(dai, NNModelDescription, toString))
        .def("__str__", &NNModelDescription::toString, DOC(dai, NNModelDescription, toString))
        .def_readwrite("model", &NNModelDescription::model, DOC(dai, NNModelDescription, model))
        .def_readwrite("platform", &NNModelDescription::platform, DOC(dai, NNModelDescription, platform))
        .def_readwrite("optimizationLevel", &NNModelDescription::optimizationLevel, DOC(dai, NNModelDescription, optimizationLevel))
        .def_readwrite("compressionLevel", &NNModelDescription::compressionLevel, DOC(dai, NNModelDescription, compressionLevel))
        .def_readwrite("snpeVersion", &NNModelDescription::snpeVersion, DOC(dai, NNModelDescription, snpeVersion))
        .def_readwrite("modelPrecisionType", &NNModelDescription::modelPrecisionType, DOC(dai, NNModelDescription, modelPrecisionType));

    // Bind SlugComponents
    slugComponents.def(py::init<>())
        .def(py::init<const std::string&, const std::string&, const std::string&, const std::string&>(),
             py::arg("teamName") = "",
             py::arg("modelSlug") = "",
             py::arg("modelVariantSlug") = "",
             py::arg("modelRef") = "")
        .def("merge", &SlugComponents::merge, DOC(dai, SlugComponents, merge))
        .def_static("split", &SlugComponents::split, py::arg("slug"), DOC(dai, SlugComponents, split))
        .def_readwrite("teamName", &SlugComponents::teamName, DOC(dai, SlugComponents, teamName))
        .def_readwrite("modelSlug", &SlugComponents::modelSlug, DOC(dai, SlugComponents, modelSlug))
        .def_readwrite("modelVariantSlug", &SlugComponents::modelVariantSlug, DOC(dai, SlugComponents, modelVariantSlug))
        .def_readwrite("modelRef", &SlugComponents::modelRef, DOC(dai, SlugComponents, modelRef));
}