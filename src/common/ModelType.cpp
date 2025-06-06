#include <depthai/common/ModelType.hpp>

namespace dai {
namespace model {

ModelType readModelType(const std::filesystem::path& modelPath) {
    auto endsWith = [](const std::filesystem::path& path, const std::string& ending) { return path.extension() == ending; };

    // == Blob ==
    if(endsWith(modelPath, ".blob")) return ModelType::BLOB;

    // == Superblob ==
    if(endsWith(modelPath, ".superblob")) return ModelType::SUPERBLOB;

    // == DLC ==
    if(endsWith(modelPath, ".dlc")) return ModelType::DLC;

    // == NNArchive ==
    if(endsWith(modelPath, ".tar")) return ModelType::NNARCHIVE;
    if(endsWith(modelPath, ".tar.gz")) return ModelType::NNARCHIVE;
    if(endsWith(modelPath, ".tar.xz")) return ModelType::NNARCHIVE;

    // == Other ==
    return ModelType::OTHER;
}

}  // namespace model
}  // namespace dai