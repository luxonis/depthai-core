#include "XLinkIn.hpp"

#include <chrono>

#include <depthai/pipeline/datatype/MessageGroup.hpp>
#include <depthai/pipeline/datatype/StreamMessageParser.hpp>
#include <memory>
#include <XLink/XLink.h>

#include "spdlog/fmt/bin_to_hex.h"
#include "spdlog/fmt/chrono.h"
#include "pipeline/NodeTiming.hpp"
#include "utilities/FrameAlignment.hpp"
#include "utilities/fmtUtils.hpp"

namespace dai {
namespace gate {

std::shared_ptr<ADatatype> convertToAccelerator(std::shared_ptr<ADatatype> msg) {
    auto imgFrame = std::dynamic_pointer_cast<dai::ImgFrame>(msg);
    if(imgFrame) {
        *imgFrame = FrameAlignment::convertImgFrame(*imgFrame);
        return imgFrame;
    }
    return msg;
}

void XLinkIn::run() {
    using namespace std::chrono_literals;
    using namespace std::chrono;
    logger->info("{} running on {}.", this->getName(), runOnHost() ? "host" : "device");

    static const milliseconds DELAY_AFTER_XLINK_CONNECTION{50};

    streamId_t streamId = XLinkOpenStream(0, properties.streamName.c_str(), 1);
    if(streamId == INVALID_STREAM_ID) {
        // if(logger) logger->error("Cannot open stream (Maximum is {} streams)",
        // X_LINK_MAX_STREAMS);
        // TODO(themarpe) - handle logging and error reporting
        return;
    }

    std::this_thread::sleep_for(DELAY_AFTER_XLINK_CONNECTION);

    while(mainLoop()) {
        using namespace std::chrono;
        auto tAbsoluteBeginning = steady_clock::now();

        auto t1Parse = steady_clock::now();
        auto t2Parse = steady_clock::now();
        StreamPacketDesc packet;

        XLinkError_t xlinkRet = XLinkReadMoveData(streamId, &packet);
        if(xlinkRet != X_LINK_SUCCESS) {
            logger->info("Read data error: {}",
                         xlinkRet);  // TODO(Morato) - move back to error, once you it is possible to detect the error was from closing the stream
            break;
        }
        logger->trace("Received message at: {}", steady_clock::now().time_since_epoch());

        t1Parse = steady_clock::now();
        const auto msg = convertToAccelerator(StreamMessageParser::parseMessage(std::move(packet)));
        t2Parse = steady_clock::now();

        std::shared_ptr<dai::MessageGroup> msgGrp = nullptr;
        if((msgGrp = std::dynamic_pointer_cast<dai::MessageGroup>(msg))) {
            for(auto& tmpMsg : msgGrp->group) {
                StreamPacketDesc packet;
                XLinkError_t xlinkRet = XLinkReadMoveData(streamId, &packet);
                if(xlinkRet != X_LINK_SUCCESS) {
                    logger->error("Read data error inside a message group: {}", xlinkRet);
                    break;
                }
                logger->trace("Received message at: {}", steady_clock::now().time_since_epoch());
                t1Parse = steady_clock::now();
                tmpMsg.second = convertToAccelerator(StreamMessageParser::parseMessage(std::move(packet)));
                t2Parse = steady_clock::now();
                if(logger->level() == spdlog::level::trace) {
                    std::vector<std::uint8_t> metadata;
                    DatatypeEnum type;
                    tmpMsg.second->serialize(metadata, type);
                }
            }
        }
        auto tGotInput = steady_clock::now();
        auto tProcessed = tGotInput;
        {
            auto blockEvent = this->outputBlockEvent();
            out.send(msg);
        }
        auto tAbsoluteEnd = steady_clock::now();
        dai::gate::logTiming(logger, this->getName(), tAbsoluteBeginning, tGotInput, tProcessed, tAbsoluteEnd);
    }
}

}  // namespace gate
}  // namespace dai
