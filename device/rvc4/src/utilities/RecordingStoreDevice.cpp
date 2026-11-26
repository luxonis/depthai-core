#include "RecordingStoreDevice.hpp"

#include <algorithm>
#include <chrono>
#include <fstream>
#include <limits>
#include <nlohmann/json.hpp>
#include <stdexcept>

#include "depthai/utility/RecordingStore.hpp"

namespace dai {
namespace gate {
namespace {

constexpr const char* MANIFEST = ".recording.json";
constexpr std::uint64_t MAX_CHUNK_BYTES = 1024 * 1024;

nlohmann::json readManifest(const std::filesystem::path& directory) {
    std::ifstream input(directory / MANIFEST);
    if(!input) throw std::runtime_error("Record missing store metadata: " + directory.filename().string());
    return nlohmann::json::parse(input);
}

void writeManifest(const std::filesystem::path& directory, const nlohmann::json& metadata) {
    const auto temporary = directory / ".recording.json.tmp";
    try {
        std::ofstream output;
        output.exceptions(std::ios::failbit | std::ios::badbit);
        output.open(temporary, std::ios::trunc);
        output << metadata.dump();
        output.close();
        std::filesystem::rename(temporary, directory / MANIFEST);
    } catch(...) {
        std::error_code error;
        std::filesystem::remove(temporary, error);
        throw;
    }
}

bool complete(const nlohmann::json& metadata) {
    const auto& streams = metadata.at("streams");
    return !streams.empty() && std::all_of(streams.begin(), streams.end(), [](const auto& closed) { return closed == true; });
}

void rejectSymlink(const std::filesystem::path& path) {
    if(std::filesystem::is_symlink(std::filesystem::symlink_status(path))) throw std::runtime_error("Record symlink is not supported: " + path.string());
}

}  // namespace

RecordingStoreDevice::RecordingStoreDevice(std::filesystem::path directory) : root(std::filesystem::absolute(std::move(directory)).lexically_normal()) {}

std::filesystem::path RecordingStoreDevice::recordingPath(const std::string& name) const {
    validateRecordingName(name);
    rejectSymlink(root);
    const auto directory = root / name;
    rejectSymlink(directory);
    if(!std::filesystem::is_directory(directory)) throw std::runtime_error("Record recording does not exist: " + name);
    return directory;
}

void RecordingStoreDevice::requireUnused(const std::string& name) const {
    const auto found = active.find(name);
    if(found != active.end() && (!found->second.streams.empty() || found->second.readers != 0)) {
        throw std::runtime_error("Record recording is in use: " + name);
    }
}

std::filesystem::path RecordingStoreDevice::reserve(const std::string& name, const std::string& stream, const void* pipeline) {
    validateRecordingName(name);
    // Legacy video stems allow trailing digits, but must still be safe path components.
    validateRecordingName(stream);
    std::lock_guard<std::mutex> lock(mutex);
    auto found = active.find(name);
    const bool shared = found != active.end() && !found->second.streams.empty() && found->second.pipeline == pipeline;
    if(!shared) {
        requireUnused(name);
        rejectSymlink(root);
        std::filesystem::create_directories(root);
        if(!std::filesystem::create_directory(root / name)) throw std::runtime_error("Record recording already exists: " + name);
        const auto now = std::chrono::system_clock::now().time_since_epoch();
        writeManifest(root / name, {{"createdMs", std::chrono::duration_cast<std::chrono::milliseconds>(now).count()}, {"streams", nlohmann::json::object()}});
        found = active.emplace(name, Active{}).first;
        found->second.pipeline = pipeline;
    }
    if(found->second.streams.count(stream) != 0) throw std::runtime_error("Record stream already used in recording: " + stream);
    auto metadata = readManifest(root / name);
    if(metadata["streams"].contains(stream)) throw std::runtime_error("Record stream already used in recording: " + stream);
    metadata["streams"][stream] = false;
    writeManifest(root / name, metadata);
    found->second.streams.emplace(stream, false);
    return root / name;
}

void RecordingStoreDevice::finish(const std::string& name, const std::string& stream) {
    std::lock_guard<std::mutex> lock(mutex);
    auto metadata = readManifest(recordingPath(name));
    metadata.at("streams").at(stream) = true;
    writeManifest(root / name, metadata);
}

void RecordingStoreDevice::claimFiles(const std::string& name, const std::vector<std::string>& filenames) {
    std::lock_guard<std::mutex> lock(mutex);
    const auto directory = recordingPath(name);
    auto& claimed = active.at(name).outputFiles;
    for(const auto& filename : filenames) {
        validateRecordingName(filename);
        if(claimed.count(filename) != 0 || std::filesystem::exists(directory / filename)) {
            throw std::runtime_error("Record video segment already exists: " + filename);
        }
    }
    claimed.insert(filenames.begin(), filenames.end());
}

void RecordingStoreDevice::release(const std::string& name, const std::string& stream) {
    std::lock_guard<std::mutex> lock(mutex);
    const auto found = active.find(name);
    if(found == active.end()) return;
    found->second.streams.erase(stream);
    if(found->second.streams.empty() && found->second.readers == 0) active.erase(found);
}

std::string RecordingStoreDevice::acquireReader(const std::filesystem::path& path) {
    std::lock_guard<std::mutex> lock(mutex);
    const auto relative = std::filesystem::weakly_canonical(path).lexically_relative(std::filesystem::weakly_canonical(root));
    if(relative.empty() || relative.is_absolute() || *relative.begin() == ".." || *relative.begin() == ".") return {};
    const auto name = relative.begin()->string();
    recordingPath(name);
    const auto found = active.find(name);
    if(found != active.end() && !found->second.streams.empty()) throw std::runtime_error("Record recording is in use: " + name);
    ++active[name].readers;
    return name;
}

void RecordingStoreDevice::releaseReader(const std::string& name) {
    if(name.empty()) return;
    std::lock_guard<std::mutex> lock(mutex);
    const auto found = active.find(name);
    if(found == active.end()) return;
    if(found->second.readers > 0) --found->second.readers;
    if(found->second.streams.empty() && found->second.readers == 0) active.erase(found);
}

std::vector<RecordingStoreDevice::File> RecordingStoreDevice::files(const std::filesystem::path& directory) const {
    std::vector<File> result;
    for(const auto& entry : std::filesystem::recursive_directory_iterator(directory)) {
        rejectSymlink(entry.path());
        if(!entry.is_regular_file()) continue;
        const auto filename = entry.path().filename().string();
        // Store bookkeeping and unfinished frames are never part of a download.
        if(filename == MANIFEST || filename == ".recording.json.tmp" || filename.rfind(".dai_closed_", 0) == 0) continue;
        if(entry.path().extension() == ".tmp") continue;
        result.emplace_back(entry.path().lexically_relative(directory).generic_string(), entry.file_size());
    }
    std::sort(result.begin(), result.end());
    return result;
}

std::vector<RecordingStoreDevice::Row> RecordingStoreDevice::list() {
    std::lock_guard<std::mutex> lock(mutex);
    std::vector<Row> result;
    rejectSymlink(root);
    if(!std::filesystem::exists(root)) return result;
    for(const auto& entry : std::filesystem::directory_iterator(root)) {
        if(entry.is_symlink() || !entry.is_directory()) continue;
        std::uint64_t bytes = 0;
        for(const auto& file : files(entry.path())) bytes += std::get<1>(file);
        std::int64_t created = 0;
        bool closed = false;
        try {
            const auto metadata = readManifest(entry.path());
            created = metadata.at("createdMs").get<std::int64_t>();
            closed = complete(metadata);
        } catch(const std::exception&) {
            // A damaged manifest or interrupted initial creation is an incomplete recording.
        }
        result.emplace_back(entry.path().filename().string(), bytes, created, closed);
    }
    std::sort(result.begin(), result.end());
    return result;
}

std::vector<RecordingStoreDevice::File> RecordingStoreDevice::listFiles(const std::string& name) {
    validateRecordingName(name);
    std::lock_guard<std::mutex> lock(mutex);
    requireUnused(name);
    return files(recordingPath(name));
}

std::vector<std::uint8_t> RecordingStoreDevice::readChunk(const std::string& name, const std::string& relative, std::uint64_t offset, std::uint64_t length) {
    validateRecordingName(name);
    const std::filesystem::path component(relative);
    if(component.empty() || component.is_absolute()) throw std::invalid_argument("Record invalid download path");
    for(const auto& part : component) {
        if(part.empty() || part == ".." || part == "." || part == MANIFEST || part == ".recording.json.tmp") {
            throw std::invalid_argument("Record invalid download path");
        }
    }
    if(length == 0 || length > MAX_CHUNK_BYTES) throw std::invalid_argument("Record invalid chunk length");
    std::lock_guard<std::mutex> lock(mutex);
    requireUnused(name);
    auto path = recordingPath(name);
    for(const auto& part : component) {
        path /= part;
        rejectSymlink(path);
    }
    if(!std::filesystem::is_regular_file(path)) throw std::runtime_error("Record download file does not exist: " + relative);
    const auto size = std::filesystem::file_size(path);
    if(offset > size || offset > static_cast<std::uint64_t>(std::numeric_limits<std::streamoff>::max())) throw std::invalid_argument("Record invalid chunk offset");
    std::vector<std::uint8_t> bytes(static_cast<std::size_t>(std::min(length, size - offset)));
    std::ifstream input(path, std::ios::binary);
    input.exceptions(std::ios::failbit | std::ios::badbit);
    input.seekg(static_cast<std::streamoff>(offset));
    if(!bytes.empty()) input.read(reinterpret_cast<char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
    return bytes;
}

void RecordingStoreDevice::remove(const std::string& name) {
    validateRecordingName(name);
    std::lock_guard<std::mutex> lock(mutex);
    requireUnused(name);
    std::filesystem::remove_all(recordingPath(name));
}

std::shared_ptr<RecordingStoreDevice> recordingStore() {
    static auto store = std::make_shared<RecordingStoreDevice>("/data/depthai/recordings");
    return store;
}

}  // namespace gate
}  // namespace dai
