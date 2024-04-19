#include "depthai/pipeline/node/host/XLinkOutHost.hpp"

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

void XLinkOutHost::setStreamName(const std::string& name) {
    streamName = name;
}

void XLinkOutHost::setConnection(std::shared_ptr<XLinkConnection> conn) {
    this->conn = std::move(conn);
}

void XLinkOutHost::run() {
    // // Create a stream for the connection
    // TODO(Morato) - automatically increase the buffer size lazily
    auto currentMaxSize = device::XLINK_USB_BUFFER_MAX_SIZE + device::XLINK_MESSAGE_METADATA_MAX_SIZE;
    XLinkStream stream(conn, streamName, currentMaxSize);
    while(isRunning()) {
        try {
            auto outgoing = in.get();
            auto metadata = StreamMessageParser::serializeMetadata(outgoing);

            using namespace std::chrono;
            // Blocking
            auto t1 = steady_clock::now();
            auto outgoingDataSize = outgoing->data->getSize();
            if(outgoingDataSize > currentMaxSize - metadata.size()) {
                logger::error("Data size {} exceeds the maximum buffer size {} - please increase the buffer size", outgoingDataSize, currentMaxSize);
                throw std::runtime_error("Data size exceeds the maximum buffer size");
            }
            if(outgoing->data->getSize() > 0) {
                stream.write(outgoing->data->getData(), metadata);
            } else {
                stream.write(metadata);
            }
            auto t2 = steady_clock::now();
            // Log
            if(spdlog::get_level() == spdlog::level::trace) {
                logger::trace("Sent message to device ({}) - data size: {}, metadata: {}, sending time: {}",
                              stream.getStreamName(),
                              outgoing->data->getSize(),
                              spdlog::to_hex(metadata),
                              duration_cast<microseconds>(t2 - t1));
            }

            // Attempt dynamic cast to MessageGroup
            if(auto msgGroupPtr = std::dynamic_pointer_cast<MessageGroup>(outgoing)) {
                logger::trace("Sending group message to device with {} messages", msgGroupPtr->group.size());
                for(auto& msg : msgGroupPtr->group) {
                    logger::trace("Sending part of a group message: {}", msg.first);
                    auto metadata = StreamMessageParser::serializeMetadata(msg.second);
                    if(msg.second->data->getSize() > 0) {
                        stream.write(msg.second->data->getData(), metadata);
                    } else {
                        stream.write(metadata);
                    }
                }
            }
        } catch(const std::exception& ex) {
            if(isRunning()) {
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
