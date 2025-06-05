#include <depthai/common/ModelType.hpp>

namespace dai {
namespace model {

ModelType readModelType(const Path& modelPath) {
    auto endsWith = [](const Path& path, const std::string& ending) { return std::filesystem::path(path.native()).extension() == ending; };

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