#include "utility/Initialization.hpp"

// std
#include <memory>

// project
#include "build/version.hpp"
#include "utility/Environment.hpp"
#include "utility/Resources.hpp"

// libraries
#include "XLink/XLink.h"
#include "spdlog/cfg/env.h"
#include "spdlog/cfg/helpers.h"
#include "spdlog/details/os.h"
#include "spdlog/spdlog.h"
extern "C" {
#include "XLink/XLinkLog.h"
}

#ifdef DEPTHAI_ENABLE_BACKWARD
    #include "backward.hpp"
#endif

// For easier access to dai namespaced symbols
namespace dai {

// Anonymous namespace to hide 'Preloader' symbol and variable as its not needed to be visible to other compilation units
namespace {

// Doing early static initialization hits this stage faster than some libraries initialize their global static members

// Preloader uses static global object constructor (works only for shared libraries)
// to execute some code upon final executable launch  or library import
// Preloader
// struct Preloader {
//     Preloader(){
//         initialize();
//     }
// } preloader;

}  // namespace

// Backward library stacktrace handling
#ifdef DEPTHAI_ENABLE_BACKWARD
static std::unique_ptr<backward::SignalHandling> signalHandler;
#endif

bool initialize() {
    return initialize(nullptr, false, nullptr);
}

bool initialize(void* javavm) {
    return initialize(nullptr, false, javavm);
}

bool initialize(std::string additionalInfo, bool installSignalHandler, void* javavm) {
    return initialize(additionalInfo.c_str(), installSignalHandler, javavm);
}

bool initialize(const char* additionalInfo, bool installSignalHandler, void* javavm) {
    // singleton for checking whether depthai was already initialized
    static const bool initialized = [&]() {
#ifdef DEPTHAI_ENABLE_BACKWARD
        // install backward if specified
        auto envSignalHandler = utility::getEnv("DEPTHAI_INSTALL_SIGNAL_HANDLER");
        if(installSignalHandler && envSignalHandler != "0") {
            signalHandler = std::make_unique<backward::SignalHandling>();
        }
#else
        (void)installSignalHandler;
#endif

        // Set global logging level from ENV variable 'DEPTHAI_LEVEL'
        // Taken from spdlog, to replace with DEPTHAI_LEVEL instead of SPDLOG_LEVEL
        // spdlog::cfg::load_env_levels();
        auto envLevel = utility::getEnv("DEPTHAI_LEVEL");
        if(!envLevel.empty()) {
            spdlog::cfg::helpers::load_levels(envLevel);
        } else {
            // Otherwise set default level to WARN
            spdlog::set_level(spdlog::level::warn);
        }

        auto debugStr = utility::getEnv("DEPTHAI_DEBUG");
        if(!debugStr.empty()) {
            // Try parsing the string as a number
            try {
                int debug{std::stoi(debugStr)};
                if(debug && (spdlog::get_level() > spdlog::level::debug)) {
                    spdlog::set_level(spdlog::level::debug);
                    spdlog::info("DEPTHAI_DEBUG enabled, lowered DEPTHAI_LEVEL to 'debug'");
                }
            } catch(const std::invalid_argument& e) {
                spdlog::warn("DEPTHAI_DEBUG value invalid: {}, should be a number (non-zero to enable)", e.what());
            }
        }

        // Print core commit and build datetime
        if(additionalInfo != nullptr && additionalInfo[0] != '\0') {
            spdlog::debug("{}", additionalInfo);
        }
        spdlog::debug(
            "Library information - version: {}, commit: {} from {}, build: {}", build::VERSION, build::COMMIT, build::COMMIT_DATETIME, build::BUILD_DATETIME);

        // Executed at library load time

        // Preload Resources (getting instance causes some internal lazy loading to start)
        Resources::getInstance();

        // Static global handler
        static XLinkGlobalHandler_t xlinkGlobalHandler = {};
        xlinkGlobalHandler.protocol = X_LINK_USB_VSC;
        xlinkGlobalHandler.options = javavm;
        auto status = XLinkInitialize(&xlinkGlobalHandler);
        std::string errorMsg;
        const auto ERROR_MSG_USB_TIP = fmt::format("If running in a container, make sure that the following is set: \"{}\"",
                                                   "-v /dev/bus/usb:/dev/bus/usb --device-cgroup-rule='c 189:* rmw'");
        if(X_LINK_SUCCESS != status) {
            std::string errorMsg = fmt::format("Couldn't initialize XLink: {}. ", XLinkErrorToStr(status));
            if(status == X_LINK_INIT_USB_ERROR) {
                errorMsg += ERROR_MSG_USB_TIP;
            }
            spdlog::debug("Initialize failed - {}", errorMsg);
            throw std::runtime_error(errorMsg);
        }
        // Check that USB protocol is available
        if(!XLinkIsProtocolInitialized(X_LINK_USB_VSC)) {
            spdlog::warn("USB protocol not available - {}", ERROR_MSG_USB_TIP);
        }

        // Suppress XLink related errors
        mvLogDefaultLevelSet(MVLOG_FATAL);

        spdlog::debug("Initialize - finished");

        return true;
    }();
    return initialized;
}

}  // namespace dai
