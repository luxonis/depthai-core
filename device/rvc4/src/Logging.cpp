#include "Logging.hpp"

#include <chrono>
#include <iostream>
#include <string>

#include "XLink/XLink.h"
#include "depthai/xlink/XLinkStream.hpp"

#include "depthai/utility/Serialization.hpp"
#include "depthai/xlink/XLinkConstants.hpp"

constexpr static auto LOGGING_BUFFER_SIZE = 512 * 1024;
constexpr static auto MAX_NUMBER_OF_LOGS = 5 * 1024;

void Logging::addLog(dai::LogMessage log) {
    std::unique_lock<std::mutex> lock(cvMutex);

    if(logBuffer.size() < MAX_NUMBER_OF_LOGS) {
        logBuffer.push_back(std::move(log));
        cv.notify_all();
    }
}

Logging::~Logging() {
    isRunning = false;
    cv.notify_all();
    if(thread.joinable()) thread.join();
}

void Logging::start() {
    logBuffer.reserve(64);

    isRunning = true;
    thread = std::thread([this]() {
        std::vector<uint8_t> buffer;
        buffer.reserve(LOGGING_BUFFER_SIZE);

        streamId_t logStreamId = XLinkOpenStream(0, dai::device::XLINK_CHANNEL_LOG, LOGGING_BUFFER_SIZE);

        while(isRunning) {
            std::vector<dai::LogMessage> logBufferSwapped;
            logBufferSwapped.reserve(64);

            buffer.clear();
            {
                std::unique_lock<std::mutex> lock(cvMutex);
                cv.wait(lock, [this]() { return !logBuffer.empty() || !isRunning; });
                if(!isRunning) break;

                logBufferSwapped.swap(logBuffer);
            }

            auto x = dai::utility::serialize(logBufferSwapped);
            buffer.insert(buffer.begin(), x.begin(), x.end());
            if(buffer.size() > LOGGING_BUFFER_SIZE) {
                buffer.clear();
                continue;
            }
            XLinkError_t ret = XLinkWriteData(logStreamId, (uint8_t*)buffer.data(), buffer.size());
            if(ret != X_LINK_SUCCESS) {
                break;
            }

            using namespace std::chrono_literals;  // Rate limit, for bigger transfers
            std::this_thread::sleep_for(10ms);
        }
    });
}
