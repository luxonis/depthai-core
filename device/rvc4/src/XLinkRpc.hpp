#pragma once

// standard
#include <thread>
#include <atomic>

// mdk
#include <XLink/XLink.h>

// project
#include "nanorpc/core/server.h"
#include "nanorpc/packer/nlohmann_msgpack.h"

// libraries
#include <spdlog/spdlog.h>

class XLinkRpc {

    nanorpc::core::server<nanorpc::packer::nlohmann_msgpack> server;
    std::string channelName;

    std::thread executorThread;
    streamId_t streamId;
    std::atomic<bool> running{true};
    std::shared_ptr<spdlog::logger> pLogger = nullptr;

public:
    // Creates an rpc on a given channel
    XLinkRpc(std::string channelName, std::shared_ptr<spdlog::logger> logger = nullptr);

    // Closes the channel, etc
    ~XLinkRpc();

    // starts xlink thread
    void start();

    // return server object
    decltype(server)& getServer() {
        return server;
    }

};