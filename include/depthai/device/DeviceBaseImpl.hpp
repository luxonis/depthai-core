#pragma once
#include <memory>
#include <mutex>
#include <vector>

#include "DeviceLogger.hpp"
#include "XLink/XLink.h"
#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/utility/Logging.hpp"
#include "nanorpc/core/client.h"
#include "nanorpc/packer/nlohmann_msgpack.h"
#include "spdlog/details/os.h"
#include "spdlog/fmt/bin_to_hex.h"
#include "spdlog/fmt/chrono.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/spdlog.h"

namespace dai {
class DeviceBaseImpl {
   public:
    DeviceBaseImpl() {
        std::cout << "Creating DeviceBaseImpl" << std::endl;
    }
    virtual ~DeviceBaseImpl();

    // TODO Copied over from the original impl
    // Default sink
    std::shared_ptr<spdlog::sinks::stdout_color_sink_mt> stdoutColorSink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    // Device Logger
    DeviceLogger logger{"host", stdoutColorSink};

    // RPC
    std::mutex rpcMutex;
    std::shared_ptr<XLinkStream> rpcStream;
    std::unique_ptr<nanorpc::core::client<nanorpc::packer::nlohmann_msgpack>> rpcClient;

    void setLogLevel(LogLevel level);
    LogLevel getLogLevel();
    void setPattern(const std::string& pattern);

    // Test for direct calls
    virtual std::vector<CameraBoardSocket> getConnectedCameras();
};
}  // namespace dai