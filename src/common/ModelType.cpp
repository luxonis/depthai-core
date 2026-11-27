#include <depthai/common/ModelType.hpp>

namespace dai {
namespace model {

ModelType readModelType(const std::filesystem::path& modelPath) {
    auto endsWith = [](const std::string& path, const std::string& ending) {
        if(ending.size() > path.size()) return false;
        return std::equal(ending.rbegin(), ending.rend(), path.rbegin());
    };

    std::string path = modelPath.string();

    if(endsWith(path, ".blob")) return ModelType::BLOB;

    if(endsWith(path, ".superblob")) return ModelType::SUPERBLOB;

    if(endsWith(path, ".dlc")) return ModelType::DLC;

    if(endsWith(path, ".tar")) return ModelType::NNARCHIVE;
    if(endsWith(path, ".tar.gz")) return ModelType::NNARCHIVE;
    if(endsWith(path, ".tar.xz")) return ModelType::NNARCHIVE;

    return ModelType::OTHER;
}

}  // namespace model
}  // namespace dai