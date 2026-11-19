#include "PtpUtils.hpp"

#include <filesystem>
#include <fstream>
#include <string>
#include <algorithm>

namespace dai {
namespace gate {

static std::string toLowerTrimmed(std::string s) {
    // Trim leading/trailing whitespace
    auto isSpace = [](unsigned char c){ return std::isspace(c); };
    s.erase(s.begin(), std::find_if_not(s.begin(), s.end(), isSpace));
    s.erase(std::find_if_not(s.rbegin(), s.rend(), isSpace).base(), s.end());

    // Lowercase everything
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c){ return std::tolower(c); });
    return s;
}

bool getPtpEnabled() {

    const std::string path = "/etc/luxonis-ptp.conf";
    std::ifstream file(path);
    if (!file.is_open()) {
        printf("PTP configuration file missing, disabling PTP.\n");
        return false;
    }

    bool enable = false;
    bool syncFrames = false;

    std::string line;
    while (std::getline(file, line)) {
        const auto lower = toLowerTrimmed(line);

        if (lower == "ptp_enable=true") {
            enable = true;
        } else if (lower == "ptp_sync_frames=true") {
            syncFrames = true;
        }
    }

    return enable && syncFrames;
}

}  // namespace gate
}  // namespace dai
