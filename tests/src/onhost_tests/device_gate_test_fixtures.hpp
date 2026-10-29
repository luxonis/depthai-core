#pragma once

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

namespace gate_test {

class FirmwareOverride {
    std::string environment;
    std::optional<std::string> previousEnvironment;
    std::filesystem::path directory;

   public:
    std::filesystem::path firmware;

    explicit FirmwareOverride(std::string environment = "DEPTHAI_DEVICE_RVC2NG_FWP") : environment(std::move(environment)) {
        if(const auto* value = std::getenv(this->environment.c_str())) previousEnvironment = value;
        auto pattern = (std::filesystem::temp_directory_path() / "depthai-rvc2ng-gate-XXXXXX").string();
        if(!mkdtemp(pattern.data())) throw std::runtime_error("Cannot create Gate firmware fixture directory");
        directory = pattern;
        firmware = directory / "firmware.tar.xz";
        std::ofstream(firmware, std::ios::binary) << "rvc2ng-test-package";
        setenv(this->environment.c_str(), firmware.c_str(), 1);
    }

    ~FirmwareOverride() {
        if(previousEnvironment) {
            setenv(environment.c_str(), previousEnvironment->c_str(), 1);
        } else {
            unsetenv(environment.c_str());
        }
        std::error_code error;
        std::filesystem::remove_all(directory, error);
    }
};

}  // namespace gate_test
