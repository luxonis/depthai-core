#include "depthai/pipeline/datatype/NeuralAssistedStereoV2Config.hpp"

namespace dai {

NeuralAssistedStereoV2Config::~NeuralAssistedStereoV2Config() = default;

void NeuralAssistedStereoV2Config::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    metadata = utility::serialize(*this);
    datatype = DatatypeEnum::NeuralAssistedStereoV2Config;
}

}  // namespace dai
