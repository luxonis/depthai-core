#pragma once

#include "depthai-shared/common/ProcessorType.hpp"
#include "depthai-shared/common/optional.hpp"
#include "depthai-shared/datatype/DatatypeEnum.hpp"
#include "depthai-shared/properties/Properties.hpp"

namespace dai {

/**
 * Specify PoolProperties options such as pool uri, pool name, ...
 */
struct PoolProperties : PropertiesSerializable<Properties, PoolProperties> {
    /**
     * Number of messages in pool
     */
    tl::optional<int> numMessages = tl::nullopt;

    /**
     * Size of data allocated for each message
     */
    tl::optional<std::int64_t> maxMessageSize = tl::nullopt;

    /**
     * Optional datatype of messages in the pool
     */
    tl::optional<DatatypeEnum> datatype = tl::nullopt;

    /**
     * Which processor should hold the pool
     */
    tl::optional<ProcessorType> processor = tl::nullopt;
};

DEPTHAI_SERIALIZE_EXT(PoolProperties, numMessages, maxMessageSize, datatype, processor);

}  // namespace dai
