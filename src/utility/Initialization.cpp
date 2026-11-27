#include "utility/Initialization.hpp"

#include <spdlog/async.h>

#include <memory>

#include "build/version.hpp"
#include "depthai/config/config.hpp"
#include "utility/Environment.hpp"
#include "utility/Logging.hpp"
#include "utility/Resources.hpp"
#ifndef DEPTHAI_DEVICE_MANAGER_ONLY
    #include "utility/Telemetry.hpp"
#endif  // DEPTHAI_DEVICE_MANAGER_ONLY
#include "XLink/XLink.h"
#include "spdlog/cfg/env.h"
#include "spdlog/cfg/helpers.h"
#include "spdlog/details/os.h"
#include "spdlog/spdlog.h"
#include "utility/Logging.hpp"
#include "utility/XLinkGlobalProfilingLogger.hpp"
extern "C" {
#include "XLink/XLinkLog.h"
}

#ifdef DEPTHAI_ENABLE_BACKWARD
    #include "backward.hpp"
#endif

namespace dai {

namespace {}  // namespace

#ifdef DEPTHAI_ENABLE_BACKWARD
static std::unique_ptr<backward::SignalHandling> signalHandler;
#endif

bool initialize() {
    return initialize(nullptr, false, nullptr);
}

bool initialize(void* javavm) {
    return initialize(nullptr, false, javavm);
}

bool initialize(const std::string& additionalInfo, bool installSignalHandler, void* javavm) {
    return initialize(additionalInfo.c_str(), installSignalHandler, javavm);
}

bool initialize(const char* additionalInfo, bool installSignalHandler, void* javavm) {
    static const bool initialized = [&]() {
        Logging::getInstance();

#ifdef DEPTHAI_ENABLE_BACKWARD
        auto envSignalHandler = utility::getEnvAs<std::string>("DEPTHAI_INSTALL_SIGNAL_HANDLER", "");
        if(installSignalHandler && envSignalHandler != "0") {
            signalHandler = std::make_unique<backward::SignalHandling>();
        }
#else
        (void)installSignalHandler;
#endif

        {
            auto javavmEnvStr = utility::getEnvAs<std::string>("DEPTHAI_LIBUSB_ANDROID_JAVAVM", "");
            if(javavm == nullptr && !javavmEnvStr.empty()) {
                sscanf(javavmEnvStr.c_str(), "%" SCNuPTR, reinterpret_cast<uintptr_t*>(&javavm));
            }
        }

        if(additionalInfo != nullptr && additionalInfo[0] != '\0') {
            logger::debug("{}", additionalInfo);
        }
        logger::debug("Library information - version: {}, commit: {} from {}, build: {}, libusb enabled: {}",
                      build::VERSION,
                      build::COMMIT,
                      build::COMMIT_DATETIME,
                      build::BUILD_DATETIME,
                      build::HAVE_LIBUSB_SUPPORT);

        Resources::getInstance();  // getInstance() starts the lazy loading of the resources

        static XLinkGlobalHandler_t xlinkGlobalHandler = {};
        xlinkGlobalHandler.protocol = X_LINK_USB_VSC;
        xlinkGlobalHandler.options = javavm;
        auto status = XLinkInitialize(&xlinkGlobalHandler);
        const auto ERROR_MSG_USB_TIP = fmt::format("If running in a container, make sure that the following is set: \"{}\"",
                                                   "-v /dev/bus/usb:/dev/bus/usb --device-cgroup-rule='c 189:* rmw'");
        if(X_LINK_SUCCESS != status) {
            std::string errorMsg = fmt::format("Couldn't initialize XLink: {}. ", XLinkErrorToStr(status));
            if(status == X_LINK_INIT_USB_ERROR) {
                errorMsg += ERROR_MSG_USB_TIP;
            }
            logger::debug("Initialize failed - {}", errorMsg);
            throw std::runtime_error(errorMsg);
        }

#ifdef DEPTHAI_ENABLE_LIBUSB
        if(!XLinkIsProtocolInitialized(X_LINK_USB_VSC)) {
            logger::warn("USB protocol not available - {}", ERROR_MSG_USB_TIP);
        }
#endif

        XLinkProfStart();
        auto profilingEnvLevel = utility::getEnvAs<std::string>("DEPTHAI_PROFILING", "");
        if(profilingEnvLevel == "1") {
            XLinkGlobalProfilingLogger::getInstance().enable(true);
        }

        // TODO(themarpe), move into XLink library
        auto xlinkEnvLevel = utility::getEnvAs<std::string>("XLINK_LEVEL", "");
        if(xlinkEnvLevel == "debug") {
            mvLogDefaultLevelSet(MVLOG_DEBUG);
        } else if(xlinkEnvLevel == "info") {
            mvLogDefaultLevelSet(MVLOG_INFO);
        } else if(xlinkEnvLevel == "warn") {
            mvLogDefaultLevelSet(MVLOG_WARN);
        } else if(xlinkEnvLevel == "error") {
            mvLogDefaultLevelSet(MVLOG_ERROR);
        } else if(xlinkEnvLevel == "fatal") {
            mvLogDefaultLevelSet(MVLOG_FATAL);
        } else if(xlinkEnvLevel == "off") {
            mvLogDefaultLevelSet(MVLOG_LAST);
        } else {
            mvLogDefaultLevelSet(MVLOG_FATAL);
        }

        logger::debug("Initialize - finished");

#ifndef DEPTHAI_DEVICE_MANAGER_ONLY
        utility::Telemetry::getInstance().init();
#endif  // DEPTHAI_DEVICE_MANAGER_ONLY

        return true;
    }();
    return initialized;
}

}  // namespace dai
