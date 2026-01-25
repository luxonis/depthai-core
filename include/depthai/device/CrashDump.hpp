#pragma once

// std
#include <cstdint>
#include <filesystem>
#include <memory>
#include <vector>

// project
#include "depthai/common/ProcessorType.hpp"
#include "depthai/device/Platform.hpp"
#include "depthai/utility/Serialization.hpp"

// third party
#include <nlohmann/json.hpp>

namespace dai {

/**
 * @brief Base class for platform-specific crash dumps
 *
 * Crash dumps are serialized as tar files with the following structure:
 * - crash_dump.tar.gz
 *   - metadata.json  (basic crash information - depthai specific)
 *   - extra.json     (user-defined extra crashdump information)
 *   - platform specific files...
 */
class CrashDump {
   public:
    virtual ~CrashDump() = default;

    /**
     * @brief Identify the platform that this crashdump corresponds to
     * @return Platform enum value
     */
    virtual Platform getPlatform() const = 0;

    /**
     * @brief Get the version of the crash dump format
     * @return Version string
     */
    virtual std::string getCrashDumpVersion() const = 0;

    /**
     * @brief Mutable access to extra data by key
     * @param key The key to access
     * @return Reference to the JSON value
     */
    nlohmann::json& operator[](const std::string& key);

    /**
     * @brief Constant access to extra data by key, throws if key doesn't exist
     * @param key The key to access
     * @return Const reference to the JSON value
     */
    const nlohmann::json& operator[](const std::string& key) const;

    /**
     * @brief Serialize the crash dump to a tar file
     * @param tarPath Path to the output tar file
     */
    virtual void toTar(const std::filesystem::path& tarPath) const = 0;

    /**
     * @brief Deserialize the crash dump from a tar file
     * @param tarPath Path to the input tar file
     */
    virtual void fromTar(const std::filesystem::path& tarPath) = 0;

    /**
     * @brief Serialize the crash dump to a byte array (bytes in filesystem after calling toTar)
     * @return Byte array
     */
    std::vector<uint8_t> toBytes() const;

    /**
     * @brief Deserialize the crash dump from a byte array (bytes in filesystem after calling toBytes)
     * @param bytes Byte array
     * @return Unique pointer to the appropriate CrashDump subclass
     */
    static std::unique_ptr<CrashDump> fromBytes(const std::vector<uint8_t>& bytes);

    /**
     * @brief Factory method to create a CrashDump from a tar file
     * @param tarPath Path to the tar file
     * @return Unique pointer to the CrashDump instance
     */
    static std::unique_ptr<CrashDump> fromTarFile(const std::filesystem::path& tarPath);

    /* Common metadata - from version.hpp */
    std::string depthaiVersion;
    std::string depthaiVersionMajor;
    std::string depthaiVersionMinor;
    std::string depthaiVersionPatch;
    std::string depthaiVersionPreReleaseType;
    std::string depthaiVersionPreReleaseVersion;
    std::string depthaiVersionBuildInfo;
    std::string depthaiCommitHash;
    std::string depthaiCommitDatetime;
    std::string depthaiBuildDatetime;
    std::string depthaiDeviceVersion;
    std::string depthaiBootloaderVersion;
    std::string depthaiDeviceRVC3Version;
    std::string depthaiDeviceRVC4Version;

    /* Device ID and timestamp of crashdump creation */
    std::string crashdumpTimestamp;
    std::string deviceId;

    /* Extra information - user-specified additional information */
    nlohmann::json extra;

   protected:
    // Writers
    void writeMetadata(nlohmann::json& json) const;

    // Readers
    void readMetadata(const nlohmann::json& json);
};

class CrashDumpRVC2 final : public CrashDump {
   public:
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

    struct CrashReportCollection {
        std::vector<CrashReport> crashReports;
        std::string depthaiCommitHash;
        std::string deviceId;
        DEPTHAI_SERIALIZE(CrashReportCollection, crashReports, depthaiCommitHash, deviceId);
    };

    CrashDumpRVC2() = default;

    /**
     * @brief Construct crashdump from a tar file
     * @param tarFile Path to the tar file
     */
    explicit CrashDumpRVC2(const std::filesystem::path& tarFile);

    Platform getPlatform() const override {
        return Platform::RVC2;
    }

    std::string getCrashDumpVersion() const override {
        return "1.0.0";
    }

    void toTar(const std::filesystem::path& tarPath) const override;
    void fromTar(const std::filesystem::path& tarPath) override;

    CrashReportCollection crashReports;
};

class CrashDumpRVC4 final : public CrashDump {
   public:
    CrashDumpRVC4() = default;

    /**
     * @brief Construct from a tar file
     * @param tarFile Path to the tar file
     */
    explicit CrashDumpRVC4(const std::filesystem::path& tarFile);

    Platform getPlatform() const override {
        return Platform::RVC4;
    }

    std::string getCrashDumpVersion() const override {
        return "1.0.0";
    }

    void toTar(const std::filesystem::path& tarPath) const override;
    void fromTar(const std::filesystem::path& tarPath) override;

    std::vector<uint8_t> data;
    std::string filename;
};

}  // namespace dai
