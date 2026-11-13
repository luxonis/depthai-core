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

}  // namespace dai::gatewire
