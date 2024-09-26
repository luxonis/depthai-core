#include <depthai/models/Models.hpp>

namespace depthai {
namespace model {

ModelType getModelType(const ModelVariant& model) {
    return std::visit([](auto&& arg) -> ModelType { return arg.type(); }, model);
}


}
}