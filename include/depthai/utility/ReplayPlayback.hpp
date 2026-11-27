#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {
namespace utility {
class BytePlayer;
}

namespace replay {

/**
 * Map an MCAP schema name to a DatatypeEnum value.
 * Throws when that type cannot be replayed.
 */
std::int32_t datatypeFromSchema(const std::string& schema);

/**
 * Read the next protobuf message from an open BytePlayer and return it as a depthai buffer.
 * Returns null at the end of the file. `datatypeValue` is the value from datatypeFromSchema.
 * The RVC4 firmware uses this so message decoding stays in one place.
 */
std::shared_ptr<Buffer> nextRecordedBuffer(utility::BytePlayer& player, std::int32_t datatypeValue);

}  // namespace replay
}  // namespace dai
