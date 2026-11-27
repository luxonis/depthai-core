#include "XLinkRpc.hpp"

#include "depthai/xlink/XLinkStream.hpp"

#include "depthai/xlink/XLinkConstants.hpp"

#include <chrono>
#include <thread>

constexpr int CACHE_LINE_SIZE = 64;
constexpr long ALIGN_UP(long S, long a) {
    return ((((S) + (a)-1)) & ~((a)-1));
};

XLinkRpc::XLinkRpc(std::string channelName, std::shared_ptr<spdlog::logger> logger) : channelName(channelName), pLogger(logger) {
    streamId = XLinkOpenStream(0, channelName.c_str(), dai::device::XLINK_USB_BUFFER_MAX_SIZE);
    if(streamId == INVALID_STREAM_ID) throw std::runtime_error("Couldn't open stream for rpc");
}

XLinkRpc::~XLinkRpc() {
    running = false;
    if(executorThread.joinable()) executorThread.join();
}

void XLinkRpc::start() {
    executorThread = std::thread([this]() {
        while(running) {
            XLinkError_t ret;
            dai::StreamPacketDesc packet;
            ret = XLinkReadMoveData(streamId, &packet);
            if(ret != X_LINK_SUCCESS) {
                break;
            }

            nanorpc::core::type::buffer buf(packet.data, packet.data + packet.length);

            nanorpc::core::type::buffer response;
            try {
                response = server.execute(std::move(buf));
            } catch(const std::exception& ex) {
                if(pLogger != nullptr) {
                    pLogger->critical("Error executing RPC call: {}", ex.what());
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
                throw;
            }

            ret = XLinkWriteData(streamId, response.data(), response.size());
            if(ret != X_LINK_SUCCESS) {
                continue;
            }
        }
    });
    // TODO(themarpe) - uncomment and test these
    // sched_param priority_param;
    // priority_param.sched_priority = 254;
    // pthread_setschedparam(executorThread.native_handle(), SCHED_RR,
    // &priority_param);
}
