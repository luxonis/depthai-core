#include <depthai/common/ModelType.hpp>

namespace dai {
namespace model {

ModelType readModelType(const std::filesystem::path& modelPath) {
    auto endsWith = [](const std::string& path, const std::string& ending) {
        if(ending.size() > path.size()) return false;
        return std::equal(ending.rbegin(), ending.rend(), path.rbegin());
    };

    // Safely turn path into a string
    std::string path = modelPath.string();

    // == Blob ==
    if(endsWith(path, ".blob")) return ModelType::BLOB;

    // == Superblob ==
    if(endsWith(path, ".superblob")) return ModelType::SUPERBLOB;

    // == DLC ==
    if(endsWith(path, ".dlc")) return ModelType::DLC;

    // == NNArchive ==
    if(endsWith(path, ".tar")) return ModelType::NNARCHIVE;
    if(endsWith(path, ".tar.gz")) return ModelType::NNARCHIVE;
    if(endsWith(path, ".tar.xz")) return ModelType::NNARCHIVE;

    // == Other ==
    return ModelType::OTHER;
}

}  // namespace model
}  // namespace dai