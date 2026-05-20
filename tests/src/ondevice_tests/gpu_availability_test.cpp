#include <catch2/catch_all.hpp>

#include <algorithm>
#include <cctype>
#include <string>

#include "depthai/device/Device.hpp"
#include "depthai/common/EepromData.hpp"
#include "depthai/device/Platform.hpp"

namespace {

int parseBoardRevisionNumber(const std::string& boardRev) {
    std::size_t pos = boardRev.find_first_of("Rr");
    if(pos == std::string::npos) pos = 0;
    else pos += 1;

    while(pos < boardRev.size() && !std::isdigit(static_cast<unsigned char>(boardRev[pos]))) {
        pos++;
    }
    if(pos >= boardRev.size()) return 0;

    int value = 0;
    while(pos < boardRev.size() && std::isdigit(static_cast<unsigned char>(boardRev[pos]))) {
        value = value * 10 + (boardRev[pos] - '0');
        pos++;
    }
    return value;
}

std::string normalizeModelName(std::string name) {
    // Normalize to the same convention used by `dai::utility::parseProductName`
    // (uppercase + spaces -> '-').
    std::transform(name.begin(), name.end(), name.begin(), [](int c) { return std::toupper(c); });
    std::replace(name.begin(), name.end(), ' ', '-');
    // Some products include a dash between OAK and 4 (e.g. "OAK-4-PRO-FF").
    // Normalize "OAK-4-*" to "OAK4-*", so model prefix checks are consistent.
    if(name.rfind("OAK-4-", 0) == 0) {
        name.erase(3, 1);  // remove '-' leaving "OAK4-..."
    }
    return name;
}

bool expectedHasGPU(dai::Device& device) {
    if(device.getPlatform() != dai::Platform::RVC4) return false;

    const auto product = normalizeModelName(device.getProductName());
    const bool isOak4D = product.size() >= 6 && product.compare(0, 6, "OAK4-D") == 0;
    const bool isOak4S = product.size() >= 6 && product.compare(0, 6, "OAK4-S") == 0;
    const bool isOak4Pro = product.size() >= 8 && product.compare(0, 8, "OAK4-PRO") == 0;
    if(!isOak4D && !isOak4S && !isOak4Pro) return false;
    if(isOak4S) return false;

    const auto eepromFactory = device.readFactoryCalibration().getEepromData();
    const auto eeprom = device.readCalibration().getEepromData();
    const std::string& boardRev = !eepromFactory.boardRev.empty() ? eepromFactory.boardRev : eeprom.boardRev;
    return parseBoardRevisionNumber(boardRev) >= 9;
}

}  // namespace

TEST_CASE("Device.hasGPU matches expected policy", "[ondevice]") {
    dai::Device device;
    const bool expected = expectedHasGPU(device);
    const bool actual = device.hasGPU();

    REQUIRE(actual == expected);
}
