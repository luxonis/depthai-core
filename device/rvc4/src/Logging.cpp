#include "Logging.hpp"

// std
#include <chrono>
#include <iostream>
#include <string>

#include "XLink/XLink.h"
#include "depthai/xlink/XLinkStream.hpp"

// shared
#include "depthai/utility/Serialization.hpp"
#include "depthai/xlink/XLinkConstants.hpp"

constexpr static auto LOGGING_BUFFER_SIZE = 512 * 1024;
constexpr static auto MAX_NUMBER_OF_LOGS = 5 * 1024;

void Logging::addLog(dai::LogMessage log) {
    // Acquire the lock (may block until current buffer is written to XLink)
    std::unique_lock<std::mutex> lock(cvMutex);

    // Append at the end of logBuffer if it's not too big
    if(logBuffer.size() < MAX_NUMBER_OF_LOGS) {
        logBuffer.push_back(std::move(log));
        // Notify others
        cv.notify_all();
    }
}

Logging::~Logging() {
    isRunning = false;
    cv.notify_all();
    if(thread.joinable()) thread.join();
}

void Logging::start() {
    // Preallocate logBuffer size for 64 messages
    logBuffer.reserve(64);

    // Start logging thread
    isRunning = true;
    thread = std::thread([this]() {
        // // Get buffer in Ddr
        std::vector<uint8_t> buffer;
        buffer.reserve(LOGGING_BUFFER_SIZE);

        streamId_t logStreamId = XLinkOpenStream(0, dai::device::XLINK_CHANNEL_LOG, LOGGING_BUFFER_SIZE);

        while(isRunning) {
            // Separate logBuffer with preallocated 64 msgs
            std::vector<dai::LogMessage> logBufferSwapped;
            logBufferSwapped.reserve(64);

            // Clear 'buffer'
            buffer.clear();
            {
                // Block until logBuffer has any data
                std::unique_lock<std::mutex> lock(cvMutex);
                cv.wait(lock, [this]() { return !logBuffer.empty() || !isRunning; });
                if(!isRunning) break;

                // cvMutex now acquired, swap logBuffers
                logBufferSwapped.swap(logBuffer);

                // 'lock' goes out of scope
            }

            // Now serialize the swapped messages
            auto x = dai::utility::serialize(logBufferSwapped);
            buffer.insert(buffer.begin(), x.begin(), x.end());
            if(buffer.size() > LOGGING_BUFFER_SIZE) {
                // Drop the buffer
                buffer.clear();
                continue;
            }
            // Send serialized 'buffer'
            XLinkError_t ret = XLinkWriteData(logStreamId, (uint8_t*)buffer.data(), buffer.size());
            if(ret != X_LINK_SUCCESS) {
                break;
            }

            // Rate limit, For bigger transfers
            using namespace std::chrono_literals;
            std::this_thread::sleep_for(10ms);
        }
    });
}
