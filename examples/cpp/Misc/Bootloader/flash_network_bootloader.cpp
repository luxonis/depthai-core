#include <algorithm>
#include <argparse/argparse.hpp>
#include <cctype>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>
#include <utility>

#include "depthai/depthai.hpp"

namespace {

constexpr int CONNECT_ATTEMPTS = 30;
constexpr auto CONNECT_RETRY_DELAY = std::chrono::seconds(1);
constexpr int VERIFY_ATTEMPTS = 30;
constexpr auto VERIFY_RETRY_DELAY = std::chrono::seconds(2);

class SafetyError : public std::runtime_error {
   public:
    using std::runtime_error::runtime_error;
};

struct BootloaderState {
    std::string version;
    bool isUserBootloader;
};

struct SelectedDevice {
    dai::DeviceInfo info;
    std::string label;
};

struct Options {
    std::string device;
    bool factory = false;
    bool skipConfirmation = false;
    std::filesystem::path bootloaderPath;
    std::optional<std::string> expectedVersion;
};

std::string deviceLabel(const dai::DeviceInfo& device) {
    if(!device.name.empty() && !device.deviceId.empty() && device.name != device.deviceId) {
        return device.name + " (" + device.deviceId + ")";
    }
    if(!device.name.empty()) return device.name;
    if(!device.deviceId.empty()) return device.deviceId;
    return device.toString();
}

bool confirm(const std::string& prompt) {
    std::cout << prompt << std::flush;

    std::string response;
    if(!std::getline(std::cin, response)) return false;

    const auto first = std::find_if_not(response.begin(), response.end(), [](unsigned char character) { return std::isspace(character); });
    const auto last = std::find_if_not(response.rbegin(), response.rend(), [](unsigned char character) { return std::isspace(character); }).base();
    if(first >= last) return false;

    response = std::string(first, last);
    std::transform(response.begin(), response.end(), response.begin(), [](unsigned char character) { return static_cast<char>(std::tolower(character)); });
    return response == "y" || response == "yes";
}

Options parseArgs(int argc, char** argv) {
    argparse::ArgumentParser program("flash_network_bootloader", "1.0.0");
    program.add_description(
        "Safely update an RVC2 device's user NETWORK bootloader using an explicit image or the image embedded in depthai-core. Factory flashing "
        "requires -f and two confirmations.");
    program.add_argument("--device", "-d")
        .default_value(std::string(""))
        .help("Device IP address, name, or device ID (defaults to the first available device)");
    program.add_argument("--factory", "-f").flag().help("Flash the factory bootloader instead of the recoverable user bootloader");
    program.add_argument("--yes", "-y").flag().help("Skip the routine user-slot confirmation (device selection and factory prompts are never skipped)");
    program.add_argument("--bootloader").help("NETWORK bootloader image to flash instead of the image embedded in depthai");
    program.add_argument("--expected-version").help("Full version expected after flashing --bootloader (for example, 0.0.28+<commit>)");

    try {
        program.parse_args(argc, argv);
    } catch(const std::runtime_error& error) {
        std::cerr << error.what() << '\n';
        std::cerr << program;
        throw;
    }

    const bool hasBootloader = program.is_used("--bootloader");
    const bool hasExpectedVersion = program.is_used("--expected-version");
    if(hasBootloader != hasExpectedVersion) {
        std::cerr << "--bootloader and --expected-version must be provided together\n";
        std::cerr << program;
        throw std::invalid_argument("--bootloader and --expected-version must be provided together");
    }

    Options options{
        program.get<std::string>("--device"),
        program.get<bool>("--factory"),
        program.get<bool>("--yes"),
        {},
        std::nullopt,
    };

    if(hasBootloader) {
        options.bootloaderPath = program.get<std::string>("--bootloader");
        options.expectedVersion = program.get<std::string>("--expected-version");
        if(!std::filesystem::is_regular_file(options.bootloaderPath)) {
            std::cerr << "Bootloader image does not exist or is not a file: " << options.bootloaderPath << '\n';
            std::cerr << program;
            throw std::invalid_argument("Bootloader image does not exist or is not a file: " + options.bootloaderPath.string());
        }
    }

    return options;
}

std::optional<SelectedDevice> selectDevice(const std::string& deviceArgument) {
    if(!deviceArgument.empty()) {
        return SelectedDevice{dai::DeviceInfo(deviceArgument), deviceArgument};
    }

    bool found = false;
    dai::DeviceInfo device;
    std::tie(found, device) = dai::DeviceBootloader::getFirstAvailableDevice();
    if(!found) {
        throw std::runtime_error("No available DepthAI devices found.");
    }

    const auto label = deviceLabel(device);
    std::cout << "First available device: " << label << '\n';
    if(!confirm("Is this the device you want to flash? [y/N] ")) return std::nullopt;

    return SelectedDevice{std::move(device), label};
}

bool confirmFactoryAccess(const SelectedDevice& device) {
    std::cout << "WARNING: Factory bootloader flashing requested for " << device.label << ".\n";
    std::cout << "A failed factory bootloader update may require hardware recovery.\n";
    return confirm("Allow factory bootloader access? [y/N] ");
}

BootloaderState inspectDevice(dai::DeviceBootloader& bootloader, bool factory = false) {
    if(bootloader.getType() != dai::DeviceBootloader::Type::NETWORK) {
        throw SafetyError("Refusing to update: the connected device is not running a NETWORK bootloader.");
    }

    if(!factory && !bootloader.isUserBootloaderSupported()) {
        throw SafetyError(
            "Refusing to update: the installed factory bootloader does not support the recoverable user bootloader slot. Use the M8/USB recovery "
            "programming board to update this device; do not flash its factory bootloader over Ethernet.");
    }

    return BootloaderState{bootloader.getVersion().toString(), bootloader.isUserBootloader()};
}

bool confirmFlash(const SelectedDevice& device, const BootloaderState& installed, const std::string& targetVersion, bool factory) {
    std::cout << "Device: " << device.label << '\n';
    std::cout << "Installed NETWORK bootloader: " << installed.version << " is user bootloader: " << (installed.isUserBootloader ? "true" : "false") << '\n';
    std::cout << "Target NETWORK bootloader:    " << targetVersion << '\n';
    if(factory) {
        std::cout << "Target: FACTORY bootloader region\n";
        std::cout << "WARNING: This operation can make the device unbootable and require hardware recovery.\n";
    } else {
        std::cout << "Target: recoverable USER bootloader slot (factory bootloader will not be modified)\n";
    }
    std::cout << "Do not disconnect the device or interrupt its power while updating.\n";
    return confirm(factory ? "Flash the factory bootloader? [y/N] " : "Continue? [y/N] ");
}

void printProgress(float progress, bool factory) {
    std::cout << "\rUpdating " << (factory ? "factory" : "user") << " bootloader: " << std::fixed << std::setw(5) << std::setprecision(1) << progress * 100.0F
              << '%' << std::flush;
}

std::unique_ptr<dai::DeviceBootloader> connectBootloader(const dai::DeviceInfo& device, bool factory) {
    std::string lastError = "device did not become available";

    for(int attempt = 0; attempt < CONNECT_ATTEMPTS; ++attempt) {
        try {
            return std::make_unique<dai::DeviceBootloader>(device, factory);
        } catch(const std::runtime_error& error) {
            if(std::string(error.what()).find("Specified device not found") == std::string::npos) throw;
            lastError = error.what();
        }

        if(attempt + 1 < CONNECT_ATTEMPTS) {
            std::cerr << "Device is reconnecting; retrying discovery (" << attempt + 1 << '/' << CONNECT_ATTEMPTS << ")...\n";
            std::this_thread::sleep_for(CONNECT_RETRY_DELAY);
        }
    }

    throw std::runtime_error("Device did not reconnect after " + std::to_string(CONNECT_ATTEMPTS) + " attempts: " + lastError);
}

void verifyUpdate(const dai::DeviceInfo& device, const std::string& targetVersion) {
    std::string lastError = "Device did not reconnect";

    for(int attempt = 0; attempt < VERIFY_ATTEMPTS; ++attempt) {
        if(attempt != 0) std::this_thread::sleep_for(VERIFY_RETRY_DELAY);

        try {
            dai::DeviceBootloader bootloader(device, false);
            const auto state = inspectDevice(bootloader);
            if(state.version != targetVersion) {
                lastError = "Device reported version " + state.version + ", expected " + targetVersion;
                continue;
            }
            if(!state.isUserBootloader) {
                lastError = "Device rebooted into its factory bootloader instead of the updated user bootloader";
                continue;
            }
            return;
        } catch(const std::exception& error) {
            lastError = error.what();
        }
    }

    throw std::runtime_error("Update was written but post-reboot verification failed: " + lastError);
}

}  // namespace

int main(int argc, char** argv) {
    Options options{};
    try {
        options = parseArgs(argc, argv);
    } catch(const std::exception&) {
        return EXIT_FAILURE;
    }

    const auto targetVersion = options.expectedVersion.value_or(dai::DeviceBootloader::getEmbeddedBootloaderVersion().toString());

    try {
        const auto selectedDevice = selectDevice(options.device);
        if(!selectedDevice) {
            std::cout << "You can select a specific device using the --device argument.\n";
            return EXIT_SUCCESS;
        }

        if(options.factory && !confirmFactoryAccess(*selectedDevice)) {
            std::cout << "Cancelled.\n";
            return EXIT_SUCCESS;
        }

        {
            auto bootloader = connectBootloader(selectedDevice->info, options.factory);
            const auto installed = inspectDevice(*bootloader, options.factory);

            if((options.factory || !options.skipConfirmation) && !confirmFlash(*selectedDevice, installed, targetVersion, options.factory)) {
                std::cout << "Cancelled.\n";
                return EXIT_SUCCESS;
            }

            const auto progressCallback = [&options](float progress) { printProgress(progress, options.factory); };
            bool success = false;
            std::string error;
            if(options.factory) {
                std::tie(success, error) = bootloader->flashBootloader(progressCallback, options.bootloaderPath);
            } else {
                std::tie(success, error) = bootloader->flashUserBootloader(progressCallback, options.bootloaderPath);
            }
            std::cout << '\n';

            if(!success) {
                throw std::runtime_error("Device rejected or failed to verify the " + std::string(options.factory ? "factory" : "user")
                                         + " bootloader update: " + error);
            }
        }

        if(options.factory) {
            std::cout << "The device reported that the factory bootloader flash completed successfully.\n";
        } else {
            std::cout << "Flash verification succeeded; waiting for the device to reboot and checking the running version...\n";
            verifyUpdate(selectedDevice->info, targetVersion);
        }
    } catch(const SafetyError& error) {
        std::cerr << error.what() << '\n';
        return 2;
    } catch(const std::exception& error) {
        std::cerr << "Failed to update the NETWORK bootloader: " << error.what() << '\n';
        if(options.factory) {
            std::cerr << "Factory bootloader flashing was enabled; inspect the device state before retrying.\n";
        } else {
            std::cerr << "The factory bootloader was not modified by this example.\n";
        }
        return EXIT_FAILURE;
    }

    if(options.factory) {
        std::cout << "NETWORK factory bootloader " << targetVersion << " was flashed successfully.\n";
    } else {
        std::cout << "NETWORK user bootloader " << targetVersion << " is running and verified.\n";
    }
    return EXIT_SUCCESS;
}
