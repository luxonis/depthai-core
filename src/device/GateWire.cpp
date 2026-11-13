#include "device/GateWire.hpp"

#ifdef DEPTHAI_ENABLE_OPENSSL
    #include <openssl/sha.h>
#endif

namespace dai::gatewire {

std::string sha256Hex(const std::vector<uint8_t>& data) {
#ifdef DEPTHAI_ENABLE_OPENSSL
    unsigned char digest[SHA256_DIGEST_LENGTH];
    SHA256(data.data(), data.size(), digest);

    constexpr char hexDigits[] = "0123456789abcdef";
    std::string hex;
    hex.reserve(2 * SHA256_DIGEST_LENGTH);
    for(unsigned char byte : digest) {
        hex.push_back(hexDigits[byte >> 4]);
        hex.push_back(hexDigits[byte & 0x0f]);
    }
    return hex;
#else
    (void)data;
    return {};
#endif
}

std::optional<std::vector<uint8_t>> parseRawFile(std::vector<uint8_t> payload, std::string& filename) {
    constexpr size_t lengthFieldSize = sizeof(uint32_t);
    if(payload.size() < lengthFieldSize) {
        return std::nullopt;
    }
    const uint32_t filenameLength = static_cast<uint32_t>(payload[0]) | static_cast<uint32_t>(payload[1]) << 8 | static_cast<uint32_t>(payload[2]) << 16
                                    | static_cast<uint32_t>(payload[3]) << 24;
    if(filenameLength > payload.size() - lengthFieldSize) {
        return std::nullopt;
    }

    const auto filenameBegin = payload.begin() + lengthFieldSize;
    const auto fileBegin = filenameBegin + filenameLength;
    filename.assign(filenameBegin, fileBegin);
    payload.erase(payload.begin(), fileBegin);
    return payload;
}

}  // namespace dai::gatewire
