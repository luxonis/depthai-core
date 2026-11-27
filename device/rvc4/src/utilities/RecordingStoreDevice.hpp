#pragma once

#include <cstdint>
#include <filesystem>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <tuple>
#include <vector>

namespace dai {
namespace gate {

// RPC tuple layouts match DeviceBase's recordStore* calls.
class RecordingStoreDevice {
   public:
    using Row = std::tuple<std::string, std::uint64_t, std::int64_t, bool>;
    using File = std::tuple<std::string, std::uint64_t>;

    explicit RecordingStoreDevice(std::filesystem::path root);
    std::filesystem::path reserve(const std::string& name, const std::string& stream, const void* pipeline);
    void finish(const std::string& name, const std::string& stream);
    void claimFiles(const std::string& name, const std::vector<std::string>& filenames);
    void release(const std::string& name, const std::string& stream);
    std::string acquireReader(const std::filesystem::path& path);
    void releaseReader(const std::string& name);
    std::vector<Row> list();
    std::vector<File> listFiles(const std::string& name);
    std::vector<std::uint8_t> readChunk(const std::string& name, const std::string& relative, std::uint64_t offset, std::uint64_t length);
    void remove(const std::string& name);

   private:
    struct Active {
        const void* pipeline = nullptr;
        std::map<std::string, bool> streams;
        std::size_t readers = 0;
        std::set<std::string> outputFiles;
    };
    const std::filesystem::path root;
    std::mutex mutex;
    std::map<std::string, Active> active;

    std::filesystem::path recordingPath(const std::string& name) const;
    void requireUnused(const std::string& name) const;
    std::vector<File> files(const std::filesystem::path& directory) const;
};

std::shared_ptr<RecordingStoreDevice> recordingStore();

}  // namespace gate
}  // namespace dai
