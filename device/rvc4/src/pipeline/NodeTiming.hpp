#pragma once

#include <spdlog/async_logger.h>

#include <chrono>
#include <memory>
#include <string>

namespace dai {
namespace gate {

inline void logTiming(const std::shared_ptr<spdlog::async_logger>& logger,
                      const std::string& nodeName,
                      std::chrono::steady_clock::time_point tAbsoluteBeginning,
                      std::chrono::steady_clock::time_point tGotInput,
                      std::chrono::steady_clock::time_point tProcessed,
                      std::chrono::steady_clock::time_point tAbsoluteEnd) {
    logger->trace("{} took {}ms, getting input {}ms, processing {}ms,  sending output {}ms",
                  nodeName,
                  std::chrono::duration_cast<std::chrono::microseconds>(tAbsoluteEnd - tAbsoluteBeginning).count() / 1000,
                  std::chrono::duration_cast<std::chrono::microseconds>(tGotInput - tAbsoluteBeginning).count() / 1000,
                  std::chrono::duration_cast<std::chrono::microseconds>(tProcessed - tGotInput).count() / 1000,
                  std::chrono::duration_cast<std::chrono::microseconds>(tAbsoluteEnd - tProcessed).count() / 1000);
}

}  // namespace gate
}  // namespace dai
