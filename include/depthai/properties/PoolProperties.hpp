#pragma once

#include "depthai/common/ProcessorType.hpp"
#include "depthai/common/optional.hpp"
#include "depthai/pipeline/datatype/DatatypeEnum.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {

/**
 * Specify PoolProperties options such as pool uri, pool name, ...
 */
struct PoolProperties : PropertiesSerializable<Properties, PoolProperties> {
    /**
     * Number of messages in pool
     */
    std::optional<int> numMessages = std::nullopt;

    /**
     * Size of data allocated for each message
     */
    std::optional<std::int64_t> maxMessageSize = std::nullopt;

    /**
     * Optional datatype of messages in the pool
     */
    std::optional<DatatypeEnum> datatype = std::nullopt;

    /**
     * Which processor should hold the pool
     */
    std::optional<ProcessorType> processor = std::nullopt;
};

DEPTHAI_SERIALIZE_EXT(PoolProperties, numMessages, maxMessageSize, datatype, processor);

}  // namespace dai
