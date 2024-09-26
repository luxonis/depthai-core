#include "ZooBindings.hpp"

// depthai
#include "depthai/modelzoo/Zoo.hpp"
#include <depthai/models/Models.hpp>
#include <depthai/models/ModelLoader.hpp>
#include <depthai/models/ModelZoo.hpp>

// Model bindings

void ZooBindings::bind(pybind11::module& m, void* pCallstack) {
    using namespace dai;

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
          py::arg("cacheDirectory") = MODEL_ZOO_DEFAULT_CACHE_DIRECTORY,
          py::arg("apiKey") = "",
          DOC(dai, getModelFromZoo));

    m.def("downloadModelsFromZoo",
          downloadModelsFromZoo,
          py::arg("path"),
          py::arg("cacheDirectory") = MODEL_ZOO_DEFAULT_CACHE_DIRECTORY,
          py::arg("apiKey") = "",
          DOC(dai, downloadModelsFromZoo));

    // Create a new submodule
    py::module m_model = m.def_submodule("model");
    py::module m_zoo = m_model.def_submodule("zoo");

    // Bind Model
    py::class_<depthai::model::Model, std::shared_ptr<depthai::model::Model>> model(m_model, "Model");
    


    py::enum_<depthai::model::ModelType>(m_model, "ModelType")
        .value("BLOB", depthai::model::ModelType::BLOB)
        .value("SUPERBLOB", depthai::model::ModelType::SUPERBLOB)
        .value("DLC", depthai::model::ModelType::DLC)
        .value("NNARCHIVE", depthai::model::ModelType::NNARCHIVE)
        .export_values();

    // Bind ModelSettings
    py::class_<depthai::model::ModelSettings, std::shared_ptr<depthai::model::ModelSettings>> modelSettings(m_model, "ModelSettings");
    modelSettings.def_readwrite("nnArchiveConfig", &depthai::model::ModelSettings::nnArchiveConfig, DOC(depthai::model, ModelSettings, settings));

    // Bind BlobSettings
    py::class_<depthai::model::BlobSettings, std::shared_ptr<depthai::model::BlobSettings>>(m_model, "BlobSettings");

    // Bind SuperBlobSettings
    py::class_<depthai::model::SuperBlobSettings, std::shared_ptr<depthai::model::SuperBlobSettings>> superblobSettings(m_model, "SuperBlobSettings");
    superblobSettings.def_readwrite("nnArchiveConfig", &depthai::model::SuperBlobSettings::nnArchiveConfig, DOC(depthai::model, SuperBlobSettings, settings));

    // Bind DlcSettings
    py::class_<depthai::model::DlcSettings, std::shared_ptr<depthai::model::DlcSettings>>(m_model, "DlcSettings");

    // Bind BlobModel
    py::class_<depthai::model::BlobModel, std::shared_ptr<depthai::model::BlobModel>> blobmodel(m_model, "BlobModel");
    blobmodel.def("settings", &depthai::model::BlobModel::settings, DOC(depthai::model, BlobModel, settings));
    // Bind SuperBlobModel
    py::class_<depthai::model::SuperBlobModel, std::shared_ptr<depthai::model::SuperBlobModel>> superblobmodel(m_model, "SuperBlobModel");
    superblobmodel.def("settings", &depthai::model::SuperBlobModel::settings, DOC(depthai::model, SuperBlobModel, settings));

    // Bind DlcModel
    py::class_<depthai::model::DlcModel, std::shared_ptr<depthai::model::DlcModel>> dlcmodel(m_model, "DlcModel");
    dlcmodel.def("settings", &depthai::model::DlcModel::settings, DOC(depthai::model, DlcModel, settings));

    // Bind load function
    m_model.def("load", py::overload_cast<const std::string&>(&depthai::model::load), py::arg("path"));
    m_zoo.def("load", &depthai::model::zoo::load, py::arg("ModelDescription"));

}
