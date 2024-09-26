#include <depthai/models/ModelLoader.hpp>
#include <depthai/nn_archive/NNArchive.hpp>

#include "../utility/ErrorMacros.hpp"
#include "models/Models.hpp"

namespace depthai {
namespace model {

/**********************************************************************************************************************/
/**********************************************************************************************************************/
/**********************************************************************************************************************/
std::unordered_map<ModelType, std::vector<std::string>> ModelLoader::getSupportedModelFileExtensions() {
    return {{ModelType::UNKNOWN, {}},
            {ModelType::BLOB, {".blob"}},
            {ModelType::SUPERBLOB, {".superblob"}},
            {ModelType::DLC, {".dlc"}},
            {ModelType::NNARCHIVE, {".tar", ".tar.gz", ".tar.xz"}}};
}

/**********************************************************************************************************************/
/**********************************************************************************************************************/
/**********************************************************************************************************************/
ModelType ModelLoader::getModelTypeFromFilePath(const std::string& path) {
    auto endsWith = [](const std::string& str, const std::string& suffix) {
        return str.size() >= suffix.size() && 0 == str.compare(str.size() - suffix.size(), suffix.size(), suffix);
    };
    for(const auto& [modelType, extensions] : getSupportedModelFileExtensions()) {
        for(const auto& extension : extensions) {
            if(endsWith(path, extension)) {
                return modelType;
            }
        }
    }
    return ModelType::UNKNOWN;
}

/**********************************************************************************************************************/
/**********************************************************************************************************************/
/**********************************************************************************************************************/
ModelType ModelLoader::getModelType() const {
    return modelType_;
}

/**********************************************************************************************************************/
/**********************************************************************************************************************/
/**********************************************************************************************************************/
ModelVariant ModelLoader::getModelVariant() const {
    DAI_CHECK_V(isModelLoaded(), "Model not loaded");
    return modelVariant_.value();
}

/**********************************************************************************************************************/
/**********************************************************************************************************************/
/**********************************************************************************************************************/
bool ModelLoader::isModelLoaded() const {
    return modelVariant_.has_value();
}

/**********************************************************************************************************************/
/**********************************************************************************************************************/
/**********************************************************************************************************************/
void ModelLoader::loadModelFromPath(const std::string& path) {
    ModelType modelType = getModelTypeFromFilePath(path);
    switch(modelType) {
        case ModelType::BLOB:
            loadBlobModel(path);
            break;
        case ModelType::SUPERBLOB:
            loadSuperblobModel(path);
            break;
        case ModelType::DLC:
            loadDlcModel(path);
            break;
        case ModelType::NNARCHIVE:
            loadNNArchive(path);
            break;
        case ModelType::UNKNOWN:
            DAI_CHECK_V(false, "Unknown model type. Check your path: " + path);
    }
}

/**********************************************************************************************************************/
/**********************************************************************************************************************/
/**********************************************************************************************************************/
void ModelLoader::loadModelFromBytes(std::vector<uint8_t> bytes, ModelType type) {
    switch(type) {
        case ModelType::BLOB:
            loadBlobModel(bytes);
            break;
        case ModelType::SUPERBLOB:
            loadSuperblobModel(bytes);
            break;
        case ModelType::DLC:
            loadDlcModel(bytes);
            break;
        case ModelType::NNARCHIVE:
            loadNNArchive(bytes);
            break;
        case ModelType::UNKNOWN:
            DAI_CHECK_V(false, "Unknown model type");
    }
}

/**********************************************************************************************************************/
/**********************************************************************************************************************/
/**********************************************************************************************************************/
void ModelLoader::loadBlobModel(std::vector<uint8_t> data) {
    DAI_CHECK_V(false, "Not implemented");
}

/**********************************************************************************************************************/
/**********************************************************************************************************************/
/**********************************************************************************************************************/
void ModelLoader::loadBlobModel(const std::string& path) {
    modelType_ = ModelType::BLOB;
    auto blobPtr = std::make_shared<dai::OpenVINO::Blob>(path);
    auto settingsPtr = std::make_shared<depthai::model::BlobSettings>();
    modelVariant_ = BlobModel(blobPtr, settingsPtr);
}

/**********************************************************************************************************************/
/**********************************************************************************************************************/
/**********************************************************************************************************************/
void ModelLoader::loadSuperblobModel(std::vector<uint8_t> data) {
    DAI_CHECK_V(false, "Not implemented");
}

/**********************************************************************************************************************/
/**********************************************************************************************************************/
/**********************************************************************************************************************/
void ModelLoader::loadSuperblobModel(const std::string& path) {
    modelType_ = ModelType::SUPERBLOB;
    auto superblobPtr = std::make_shared<dai::OpenVINO::SuperBlob>(path);
    auto settingsPtr = std::make_shared<depthai::model::SuperBlobSettings>();
    modelVariant_ = SuperBlobModel(superblobPtr, settingsPtr);
}

/**********************************************************************************************************************/
/**********************************************************************************************************************/
/**********************************************************************************************************************/
void ModelLoader::loadDlcModel(std::vector<uint8_t> data) {
    DAI_CHECK_V(false, "Not implemented");
}

/**********************************************************************************************************************/
/**********************************************************************************************************************/
/**********************************************************************************************************************/
void ModelLoader::loadDlcModel(const std::string& path) {
    modelType_ = ModelType::DLC;
    auto settingsPtr = std::make_shared<depthai::model::DlcSettings>();
    modelVariant_ = DlcModel(path, settingsPtr);
}

/**********************************************************************************************************************/
/**********************************************************************************************************************/
/**********************************************************************************************************************/
void ModelLoader::loadNNArchive(std::vector<uint8_t> data) {
    DAI_CHECK_V(false, "Not implemented");
}

/**********************************************************************************************************************/
/**********************************************************************************************************************/
/**********************************************************************************************************************/
void ModelLoader::loadNNArchive(const std::string& path) {
    // TODO: Remove NNArchive type completely == Load data from NNArchive but return a ModelVariant, not an NNArchive object
    dai::NNArchive archive(path);
    switch(archive.getModelType()) {
        case dai::model::ModelType::BLOB:
            modelType_ = ModelType::BLOB;
            {
                auto blobPtr = std::make_shared<dai::OpenVINO::Blob>(archive.getBlob().value().data);
                auto settingsPtr = std::make_shared<depthai::model::BlobSettings>();
                settingsPtr->nnArchiveConfig = archive.getVersionedConfig();
                modelVariant_ = BlobModel(blobPtr, settingsPtr);
            }
            break;
        case dai::model::ModelType::SUPERBLOB:
            modelType_ = ModelType::SUPERBLOB;
            {
                auto superblobPtr = std::make_shared<dai::OpenVINO::SuperBlob>(archive.getSuperBlob().value());
                auto settingsPtr = std::make_shared<depthai::model::SuperBlobSettings>();
                settingsPtr->nnArchiveConfig = archive.getVersionedConfig();
                modelVariant_ = SuperBlobModel(superblobPtr, settingsPtr);
            }
            break;
        case dai::model::ModelType::DLC:
        case dai::model::ModelType::OTHER:
        case dai::model::ModelType::NNARCHIVE:
            DAI_CHECK_V(false, "Unsupported model type");
    }
}

/**********************************************************************************************************************/
/**********************************************************************************************************************/
/**********************************************************************************************************************/
ModelVariant load(const std::string& modelPath) {
    ModelLoader loader;
    loader.loadModelFromPath(modelPath);
    return loader.getModelVariant();
}

/**********************************************************************************************************************/
/**********************************************************************************************************************/
/**********************************************************************************************************************/
ModelVariant load(const std::vector<uint8_t>& data, ModelType type) {
    ModelLoader loader;
    loader.loadModelFromBytes(data, type);
    DAI_CHECK_V(loader.isModelLoaded(), "Model not loaded");
    return loader.getModelVariant();
}

}  // namespace model
}  // namespace depthai