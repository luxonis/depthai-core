#include "depthai/pipeline/node/BenchmarkIn.hpp"

#include <chrono>

#include "depthai/pipeline/datatype/BenchmarkReport.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"
namespace dai {
namespace node {

void BenchmarkIn::sendReportEveryNMessages(uint32_t num) {
    properties.reportEveryNMessages = num;
}

void BenchmarkIn::setRunOnHost(bool runOnHost) {
    runOnHostVar = runOnHost;
}

bool BenchmarkIn::runOnHost() const {
    return runOnHostVar;
}

void BenchmarkIn::logReportsAsWarnings(bool logReportsAsWarnings) {
    properties.logReportsAsWarnings = logReportsAsWarnings;
}

void BenchmarkIn::measureIndividualLatencies(bool attachLatencies) {
    properties.attachLatencies = attachLatencies;
}

void BenchmarkIn::run() {
    auto& logger = pimpl->logger;
    using namespace std::chrono;

    uint32_t numMessages = properties.reportEveryNMessages;

    // Decide if we will store latencies or not
    bool storeLatencies = false;
    if(properties.attachLatencies) {
        if(numMessages <= 1000) {
            // We'll store latencies for this batch
            storeLatencies = true;
        } else {
            // Warn upfront if user wanted latencies but # messages is too high
            logger->warn("Number of messages > 1000, latencies not individually attached.");
        }
    }

    uint32_t messageCount = 0;
    float totalLatency = 0.0f;

    std::vector<float> latencies;
    if(storeLatencies) {
        latencies.reserve(numMessages);
    }

    auto start = steady_clock::now();

    while(mainLoop()) {
        std::shared_ptr<dai::Buffer> inMessage = nullptr;
        std::shared_ptr<dai::BenchmarkReport> reportMessage = nullptr;
        {
            auto blockEvent = this->inputBlockEvent();
            inMessage = input.get<dai::Buffer>();
        }

        // If this is the first message of the batch, reset counters
        if(messageCount == 0) {
            start = steady_clock::now();
            totalLatency = 0.0f;

            // Clear vector if we are storing latencies
            if(storeLatencies) {
                latencies.clear();
                latencies.reserve(numMessages);
            }
        }

        if(messageCount < numMessages) {
            auto currentTs = steady_clock::now();
            auto messageTs = steady_clock::now();
            if(runOnHostVar) {
                messageTs = inMessage->getTimestamp();
            } else {
                messageTs = inMessage->getTimestampDevice();
            }

            duration<float> diff = currentTs - messageTs;
            logger->trace("Frame latency: {} s", diff.count());

            // Accumulate for average
            totalLatency += diff.count();

            // Optionally store individual latencies
            if(storeLatencies) {
                latencies.push_back(diff.count());
            }

            messageCount++;
        } else {
            // We reached our batch size, so time to compute and send the report
            auto stop = steady_clock::now();
            duration<float> durationS = stop - start;

            reportMessage = std::make_shared<dai::BenchmarkReport>();
            reportMessage->numMessagesReceived = numMessages;
            reportMessage->timeTotal = durationS.count();
            reportMessage->fps = numMessages / durationS.count();
            reportMessage->averageLatency = totalLatency / numMessages;

            // Attach latencies only if we're storing them
            if(storeLatencies) {
                reportMessage->latencies = latencies;
            }

            // Decide how to log (warn or info) once, then do all the logs
            auto logFunc = [&](auto fmt, auto... args) {
                if(properties.logReportsAsWarnings) {
                    logger->warn(fmt, std::forward<decltype(args)>(args)...);
                } else {
                    logger->trace(fmt, std::forward<decltype(args)>(args)...);
                }
            };

            // Unconditional logs, using chosen severity
            logFunc("FPS: {}", reportMessage->fps);
            logFunc("Messages took {} s", reportMessage->timeTotal);
            logFunc("Average latency: {} s", reportMessage->averageLatency);

            // Reset for next batch
            messageCount = 0;
        }

        {
            auto blockEvent = this->outputBlockEvent();

            if(reportMessage) {
                // Send out the report
                report.send(reportMessage);
                logger->trace("Sent report message");
            }

            // Passthrough the message
            passthrough.send(inMessage);
        }
    }
}

}  // namespace node
}  // namespace dai
