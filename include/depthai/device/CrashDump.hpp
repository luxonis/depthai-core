#pragma once

// std
#include <cstdint>
#include <unordered_map>

// project
#include "depthai/common/ProcessorType.hpp"
#include "depthai/common/optional.hpp"
#include "depthai/utility/Serialization.hpp"

namespace dai {

struct CrashDump {
    struct CrashReport {
        ProcessorType processor;
        std::string errorSource;
        uint32_t crashedThreadId = 0;

        struct ErrorSourceInfo {
            struct AssertContext {
                std::string fileName;
                std::string functionName;
                uint32_t line = 0;
                DEPTHAI_SERIALIZE(AssertContext, fileName, functionName, line);
            };

            AssertContext assertContext;

            struct TrapContext {
                uint32_t trapNumber = 0;
                uint32_t trapAddress = 0;
                std::string trapName;
                DEPTHAI_SERIALIZE(TrapContext, trapNumber, trapAddress, trapName);
            };

            TrapContext trapContext;

            uint32_t errorId = 0;

            DEPTHAI_SERIALIZE(ErrorSourceInfo, assertContext, trapContext, errorId);
        };

        ErrorSourceInfo errorSourceInfo;

        struct ThreadCallstack {
            uint32_t threadId = 0;
            std::string threadName;
            std::string threadStatus;
            uint32_t stackBottom = 0;
            uint32_t stackTop = 0;
            uint32_t stackPointer = 0;
            uint32_t instructionPointer = 0;

            struct CallstackContext {
                uint32_t callSite = 0;
                uint32_t calledTarget = 0;
                uint32_t framePointer = 0;
                std::string context;
                DEPTHAI_SERIALIZE(CallstackContext, callSite, calledTarget, framePointer, context);
            };

            std::vector<CallstackContext> callStack;

            DEPTHAI_SERIALIZE(ThreadCallstack, threadId, threadName, threadStatus, stackBottom, stackTop, stackPointer, instructionPointer, callStack);
        };

        std::vector<ThreadCallstack> threadCallstack;
        DEPTHAI_SERIALIZE(CrashReport, processor, errorSource, crashedThreadId, errorSourceInfo, threadCallstack);
    };

    std::vector<CrashReport> crashReports;
    std::string depthaiCommitHash;
    std::string deviceId;

    nlohmann::json serializeToJson() const {
        std::vector<std::uint8_t> data;
        utility::serialize<SerializationType::JSON>(*this, data);
        return nlohmann::json::parse(data);
    }
};

DEPTHAI_SERIALIZE_EXT(CrashDump, crashReports, depthaiCommitHash, deviceId);

}  // namespace dai
