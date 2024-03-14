#include "depthai/pipeline/node/host/XLinkInHost.hpp"

#include "depthai/pipeline/datatype/StreamMessageParser.hpp"
#include "depthai/xlink/XLinkConnection.hpp"
#include "depthai/xlink/XLinkConstants.hpp"
#include "depthai/xlink/XLinkStream.hpp"
#include "spdlog/fmt/bin_to_hex.h"
#include "spdlog/fmt/chrono.h"

// libraries
#include "depthai/pipeline/datatype/MessageGroup.hpp"
#include "utility/Logging.hpp"

namespace dai {
namespace node {
// XLinkInHost::XLinkInHost(std::shared_ptr<XLinkConnection> conn, std::string streamName) : conn(std::move(conn)), streamName(std::move(streamName)){};

void XLinkInHost::setStreamName(const std::string& name) {
    streamName = name;
}

void XLinkInHost::setConnection(std::shared_ptr<XLinkConnection> conn) {
    this->conn = std::move(conn);
}

void XLinkInHost::run() {
    // Create a stream for the connection
    XLinkStream stream(std::move(conn), streamName, 1);
    while(isRunning()) {
        try {
            // Blocking -- parse packet and gather timing information
            auto packet = stream.readMove();
            const auto t1Parse = std::chrono::steady_clock::now();
            const auto msg = StreamMessageParser::parseMessage(std::move(packet));
            if(std::dynamic_pointer_cast<MessageGroup>(msg) != nullptr) {
                auto msgGrp = std::static_pointer_cast<MessageGroup>(msg);
                for(auto& msg : msgGrp->group) {
                    auto dpacket = stream.readMove();
                    msg.second = StreamMessageParser::parseMessage(&dpacket);
                }
            }
            const auto t2Parse = std::chrono::steady_clock::now();

            // Trace level debugging
            if(logger::get_level() == spdlog::level::trace) {
                std::vector<std::uint8_t> metadata;
                DatatypeEnum type;
                msg->serialize(metadata, type);
                logger::trace("Received message from device ({}) - parsing time: {}, data size: {}, object type: {} object data: {}",
                              streamName,
                              std::chrono::duration_cast<std::chrono::microseconds>(t2Parse - t1Parse),
                              msg->data->getSize(),
                              static_cast<std::int32_t>(type),
                              spdlog::to_hex(metadata));
            }

            out.send(msg);
            // // Add 'data' to queue
            // if(!queue.push(msg)) {
            //     throw std::runtime_error(fmt::format("Underlying queue destructed"));
            // }

            // Call callbacks
            // {
            //     std::unique_lock<std::mutex> l(callbacksMtx);
            //     for(const auto& kv : callbacks) {
            //         const auto& callback = kv.second;
            //         try {
            //             callback(name, msg);
            //         } catch(const std::exception& ex) {
            //             logger::error("Callback with id: {} throwed an exception: {}", kv.first, ex.what());
            //         }
            //     }
            // }
        } catch(const std::exception& ex) {
            if(isRunning()){
                auto exceptionMessage = fmt::format("Communication exception - possible device error/misconfiguration. Original message '{}'", ex.what());
                throw std::runtime_error(exceptionMessage);
            } else {
                // If the node is not running, we can safely ignore the exception, since it's expected
                logger::info("XLinkInHost node stopped - exception: {}", ex.what());
                break;
            }
        }
    }
}

}  // namespace node
}  // namespace dai