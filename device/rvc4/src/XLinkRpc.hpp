#pragma once

#include <thread>
#include <atomic>

#include <XLink/XLink.h>

#include "nanorpc/core/server.h"
#include "nanorpc/packer/nlohmann_msgpack.h"

#include <spdlog/spdlog.h>

class XLinkRpc {

    nanorpc::core::server<nanorpc::packer::nlohmann_msgpack> server;
    std::string channelName;

    std::thread executorThread;
    streamId_t streamId;
    std::atomic<bool> running{true};
    std::shared_ptr<spdlog::logger> pLogger = nullptr;

   public:
    XLinkRpc(std::string channelName, std::shared_ptr<spdlog::logger> logger = nullptr);

    ~XLinkRpc();

    void start();

    decltype(server)& getServer() {
        return server;
    }
};