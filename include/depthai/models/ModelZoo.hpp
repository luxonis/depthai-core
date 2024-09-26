#pragma once

#include <depthai/models/ModelLoader.hpp>
#include <depthai/models/Models.hpp>
#include <depthai/modelzoo/Zoo.hpp>

namespace depthai {
namespace model {
namespace zoo {

ModelVariant load(const dai::NNModelDescription& description) {
    std::string path = getModelFromZoo(description);
    return depthai::model::load(path);
}

}  // namespace zoo
}  // namespace model
}  // namespace depthai