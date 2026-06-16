#include "utility/Uuid.hpp"

#include <array>
#include <chrono>
#include <cstdint>
#include <iomanip>
#include <random>
#include <sstream>

namespace dai {
namespace utility {

std::string generateUuidV7() {
    static std::mt19937_64 generator(std::random_device{}());
    static std::uniform_int_distribution<int> distribution(0, 255);

    std::array<std::uint8_t, 16> bytes{};
    const auto nowMs = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

    bytes[0] = static_cast<std::uint8_t>((nowMs >> 40) & 0xFF);
    bytes[1] = static_cast<std::uint8_t>((nowMs >> 32) & 0xFF);
    bytes[2] = static_cast<std::uint8_t>((nowMs >> 24) & 0xFF);
    bytes[3] = static_cast<std::uint8_t>((nowMs >> 16) & 0xFF);
    bytes[4] = static_cast<std::uint8_t>((nowMs >> 8) & 0xFF);
    bytes[5] = static_cast<std::uint8_t>(nowMs & 0xFF);

    for(std::size_t index = 6; index < bytes.size(); ++index) {
        bytes[index] = static_cast<std::uint8_t>(distribution(generator));
    }

    bytes[6] = static_cast<std::uint8_t>((bytes[6] & 0x0F) | 0x70);
    bytes[8] = static_cast<std::uint8_t>((bytes[8] & 0x3F) | 0x80);

    std::ostringstream stream;
    stream << std::hex << std::setfill('0');
    for(std::size_t index = 0; index < bytes.size(); ++index) {
        stream << std::setw(2) << static_cast<int>(bytes[index]);
        if(index == 3 || index == 5 || index == 7 || index == 9) {
            stream << '-';
        }
    }
    return stream.str();
}

}  // namespace utility
}  // namespace dai
