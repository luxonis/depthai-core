#include "depthai/pipeline/SideChannel.hpp"

#include <depthai/pipeline/datatype/StreamMessageParser.hpp>
#include <iostream>

#include "XLink/XLink.h"
#include "depthai-shared/xlink/XLinkConstants.hpp"
#include "depthai/xlink/XLinkStream.hpp"

namespace dai {

const std::chrono::milliseconds DELAY_AFTER_XLINK_CONNECTION{50};
const bool PRINT_DEBUG = false;

SideChannel::SideChannel() {
    std::cout << "Creating a side channel" << std::endl;
}

void SideChannel::sendMessage(const std::shared_ptr<dai::ADatatype>& message) {
    lockingQueue.push(message);
}

void SideChannel::stop() {
    running = false;
    lockingQueue.destruct();
    if(thread.joinable()) {
        thread.join();
    }
}

void SideChannel::start() {
    std::cout << "Starting the side channel!" << std::endl;
    running = true;
    thread = std::thread(&SideChannel::threadedRun, this);
}

void SideChannel::threadedRun() {
    int currentStreamSize = dai::device::XLINK_USB_BUFFER_MAX_SIZE;
    streamId_t streamId = XLinkOpenStream(0, dai::device::XLINK_CHANNEL_SIDE_CHANNEL, currentStreamSize);
    if(streamId == INVALID_STREAM_ID) {
        std::cerr << "Invalid stream id in side channel" << std::endl;  // TODO
        return;
    }

    // Sleep for constant time
    std::this_thread::sleep_for(DELAY_AFTER_XLINK_CONNECTION);

    while(running) {
        std::shared_ptr<ADatatype> message = nullptr;
        if(!lockingQueue.waitAndPop(message)) {
            running = false;
            std::cout << "Stopping the side channel" << std::endl;
            break;
        }

        // Serialize and get metadata
        auto serialized = message->serialize();
        std::vector<uint8_t> metadata = StreamMessageParser::serializeMetadata(*serialized.metadata);
        uint32_t metadataSize = metadata.size();

        if(PRINT_DEBUG) {
            printf("metadata (correct): ");
            for(uint8_t b : metadata) {
                printf("%02X ", b);
            }
            printf("\n");
        }

        // 'success' being false only indicates metadata buffer (metadataSize >
        // metadataMaxSize) running out. For that case, add a log instead,
        // specifying an error of the message being dropped
        // if(metadata.size() > metadataMaxSize) {
        //     logger->error("Message has too much metadata ({}B) to serialize. Maximum is {}B. Dropping message", metadata.size(), metadataMaxSize);
        //     continue;
        // }

        long messageSize = (long)(serialized.data->getSize() + metadata.size());

        if(PRINT_DEBUG) {
            printf(
                "PlgXLinkOut | metadataSize: %zu, PoBuf::size: %d, stream name: "
                "%s\n",
                (size_t)metadata.size(),
                (int)serialized.data->getSize(),
                dai::device::XLINK_CHANNEL_SIDE_CHANNEL);
        }

        // Write data
        if(serialized.data->getSize() > 0 || metadata.size() > 0) {
            // First check if message size is less or equal to currentStreamSize
            // if not, resize stream
            // TODO(themarpe) - this might not be a stable solution
            // but is only used for cases where hint technique would fail
            // if(messageSize > currentStreamSize) {
            //     // if(PRINT_DEBUG) printf("PlgXLinkOut | Increasing stream ('%s') size
            //     // - streamId before: %lu, currentStreamSize before: %d\n",
            //     // streamName.c_str(), streamId, currentStreamSize);

            //     // Resize to PoBuf::size + metadataMaxSize instead of current metadata
            //     int newStreamSize = serialized.data->getData().size() + metadataMaxSize;

            //     // TODO(themarpe) - port
            //     logger->info("Increasing '{}' stream size from {}B to {}B", streamName, currentStreamSize, newStreamSize);

            //     // Increase stream size, by requesting another 'open stream' request
            //     streamId = XLinkOpenStream(0, streamName.c_str(), newStreamSize);
            //     currentStreamSize = newStreamSize;
            // }

            // PoBuf::size may be 0 OR metadataOnly property is set
            XLinkError_t status = X_LINK_SUCCESS;
            // auto t3 = steady_clock::now();
            if(serialized.data->getSize() == 0) {
                // Send only metadata
                status = XLinkWriteData(streamId, metadata.data(), metadata.size());
            } else {
                // PoBuf is sent seperatelly, and metadata separatelly (in one xlink packet still)
                std::vector<std::uint8_t> tmp(serialized.data->getData().begin(), serialized.data->getData().end());
                // status = XLinkWriteData2(streamId, (uint8_t*)serialized.data->getData().data(), serialized.data->getData().size(), metadata.data(),
                // metadata.size());
                status = XLinkWriteData2(streamId, tmp.data(), tmp.size(), metadata.data(), metadata.size());
            }
            // auto t4 = steady_clock::now();
            // if(logger->level() == spdlog::level::trace) {
            //     logger->trace("Sent message from device ({}) - parsing time: {}, data size: {}, metadata: {}, sending time: {}",
            //                   properties.streamName,
            //                   duration_cast<microseconds>(t2 - t1),
            //                   serialized.data->getSize(),
            //                   spdlog::to_hex(metadata),
            //                   duration_cast<microseconds>(t4 - t3));
            // }

            if(status == X_LINK_SUCCESS) {
                // packetCounterInSecond++;
                if(PRINT_DEBUG) {
                    printf(
                        "PlgXLinkOut | Sent packet, PoBuf size: %d, metadataSize: "
                        "%d, stream name: %s\n",
                        (int)serialized.data->getSize(),
                        (int)metadata.size(),
                        dai::device::XLINK_CHANNEL_SIDE_CHANNEL);
                }
            } else {
                using namespace std::chrono_literals;
                if(PRINT_DEBUG) printf("PlgXLinkOut | write error, status: %d\n", status);
                std::this_thread::sleep_for(10ms);
                continue;
            }
        }
    }
}

}  // namespace dai
