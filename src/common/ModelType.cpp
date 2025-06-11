#include <depthai/common/ModelType.hpp>

namespace dai {
namespace model {

ModelType readModelType(const std::string& modelPath) {
    auto endsWith = [](const std::string& path, const std::string& ending) {
        if(ending.size() > path.size()) return false;
        return std::equal(ending.rbegin(), ending.rend(), path.rbegin());
    };

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