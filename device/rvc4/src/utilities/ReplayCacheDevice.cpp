#include "ReplayCacheDevice.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <mutex>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace dai {
namespace gate {
namespace {

constexpr const char* kCacheDir = "/data/depthai/replay_cache";
constexpr std::uint64_t kDefaultLimit = 32ull << 30;

class Sha256 {
   public:
    Sha256() {
        state[0] = 0x6a09e667u;
        state[1] = 0xbb67ae85u;
        state[2] = 0x3c6ef372u;
        state[3] = 0xa54ff53au;
        state[4] = 0x510e527fu;
        state[5] = 0x9b05688cu;
        state[6] = 0x1f83d9abu;
        state[7] = 0x5be0cd19u;
    }

    void update(const std::uint8_t* data, std::size_t len) {
        for(std::size_t i = 0; i < len; i++) {
            buffer[bufferLen++] = data[i];
            if(bufferLen == 64) {
                transform(buffer);
                bitLen += 512;
                bufferLen = 0;
            }
        }
    }

    std::string hex() {
        const std::uint64_t totalBits = bitLen + static_cast<std::uint64_t>(bufferLen) * 8u;
        buffer[bufferLen++] = 0x80u;
        if(bufferLen > 56) {
            while(bufferLen < 64) buffer[bufferLen++] = 0;
            transform(buffer);
            bufferLen = 0;
        }
        while(bufferLen < 56) buffer[bufferLen++] = 0;
        for(int i = 7; i >= 0; i--) buffer[bufferLen++] = static_cast<std::uint8_t>((totalBits >> (i * 8)) & 0xffu);
        transform(buffer);
        static const char* kHex = "0123456789abcdef";
        std::string out(64, '0');
        for(int i = 0; i < 8; i++) {
            for(int b = 0; b < 4; b++) {
                const auto value = static_cast<std::uint8_t>((state[i] >> (24 - b * 8)) & 0xffu);
                out[static_cast<std::size_t>(i * 8 + b * 2)] = kHex[value >> 4];
                out[static_cast<std::size_t>(i * 8 + b * 2 + 1)] = kHex[value & 0x0f];
            }
        }
        return out;
    }

   private:
    static std::uint32_t rotr(std::uint32_t value, std::uint32_t bits) {
        return (value >> bits) | (value << (32 - bits));
    }

    void transform(const std::uint8_t chunk[64]) {
        static const std::uint32_t k[64] = {
            0x428a2f98u, 0x71374491u, 0xb5c0fbcfu, 0xe9b5dba5u, 0x3956c25bu, 0x59f111f1u, 0x923f82a4u, 0xab1c5ed5u, 0xd807aa98u, 0x12835b01u, 0x243185beu,
            0x550c7dc3u, 0x72be5d74u, 0x80deb1feu, 0x9bdc06a7u, 0xc19bf174u, 0xe49b69c1u, 0xefbe4786u, 0x0fc19dc6u, 0x240ca1ccu, 0x2de92c6fu, 0x4a7484aau,
            0x5cb0a9dcu, 0x76f988dau, 0x983e5152u, 0xa831c66du, 0xb00327c8u, 0xbf597fc7u, 0xc6e00bf3u, 0xd5a79147u, 0x06ca6351u, 0x14292967u, 0x27b70a85u,
            0x2e1b2138u, 0x4d2c6dfcu, 0x53380d13u, 0x650a7354u, 0x766a0abbu, 0x81c2c92eu, 0x92722c85u, 0xa2bfe8a1u, 0xa81a664bu, 0xc24b8b70u, 0xc76c51a3u,
            0xd192e819u, 0xd6990624u, 0xf40e3585u, 0x106aa070u, 0x19a4c116u, 0x1e376c08u, 0x2748774cu, 0x34b0bcb5u, 0x391c0cb3u, 0x4ed8aa4au, 0x5b9cca4fu,
            0x682e6ff3u, 0x748f82eeu, 0x78a5636fu, 0x84c87814u, 0x8cc70208u, 0x90befffau, 0xa4506cebu, 0xbef9a3f7u, 0xc67178f2u};
        std::uint32_t w[64];
        for(int i = 0; i < 16; i++) {
            w[i] = (std::uint32_t(chunk[i * 4]) << 24) | (std::uint32_t(chunk[i * 4 + 1]) << 16) | (std::uint32_t(chunk[i * 4 + 2]) << 8)
                   | std::uint32_t(chunk[i * 4 + 3]);
        }
        for(int i = 16; i < 64; i++) {
            const auto s0 = rotr(w[i - 15], 7) ^ rotr(w[i - 15], 18) ^ (w[i - 15] >> 3);
            const auto s1 = rotr(w[i - 2], 17) ^ rotr(w[i - 2], 19) ^ (w[i - 2] >> 10);
            w[i] = w[i - 16] + s0 + w[i - 7] + s1;
        }
        auto a = state[0];
        auto b = state[1];
        auto c = state[2];
        auto d = state[3];
        auto e = state[4];
        auto f = state[5];
        auto g = state[6];
        auto h = state[7];
        for(int i = 0; i < 64; i++) {
            const auto s1 = rotr(e, 6) ^ rotr(e, 11) ^ rotr(e, 25);
            const auto ch = (e & f) ^ ((~e) & g);
            const auto temp1 = h + s1 + ch + k[i] + w[i];
            const auto s0 = rotr(a, 2) ^ rotr(a, 13) ^ rotr(a, 22);
            const auto maj = (a & b) ^ (a & c) ^ (b & c);
            const auto temp2 = s0 + maj;
            h = g;
            g = f;
            f = e;
            e = d + temp1;
            d = c;
            c = b;
            b = a;
            a = temp1 + temp2;
        }
        state[0] += a;
        state[1] += b;
        state[2] += c;
        state[3] += d;
        state[4] += e;
        state[5] += f;
        state[6] += g;
        state[7] += h;
    }

    std::uint32_t state[8]{};
    std::uint64_t bitLen = 0;
    std::uint8_t buffer[64]{};
    std::size_t bufferLen = 0;
};

struct Entry {
    std::string key;
    std::int32_t kind = 0;
    std::string sourceHash;
    std::uint64_t sizeBytes = 0;
    std::int64_t lastUseMs = 0;
    int armed = 0;
    int prepared = 0;
    bool persistent = false;
    bool started = false;
};

struct DirectoryUpload {
    std::map<std::string, std::uint64_t> files;
    std::map<std::string, std::uint64_t> written;
    std::uint64_t writtenBytes = 0;
};

struct Book {
    std::uint64_t limitBytes = kDefaultLimit;
    std::vector<Entry> entries;
    std::unordered_map<std::string, std::uint64_t> pendingBytes;
    std::unordered_map<std::string, int> pendingBucket;
    std::unordered_map<std::string, DirectoryUpload> pendingDirectories;
    std::unordered_set<std::string> pendingPrepared;
    std::unordered_map<std::string, std::uint64_t> pendingWritten;
    bool loaded = false;
};

Book gBook;
std::unordered_map<std::string, int> gScratchLeases;
std::mutex gMutex;

std::filesystem::path cacheDir() {
    return kCacheDir;
}

std::string safeName(const std::string& key) {
    std::string cleaned;
    cleaned.reserve(key.size());
    for(const char character : key) {
        const auto asUnsigned = static_cast<unsigned char>(character);
        if(std::isalnum(asUnsigned) || character == '_' || character == '-' || character == '.')
            cleaned.push_back(character);
        else
            cleaned.push_back('_');
    }
    if(cleaned.empty()) throw std::runtime_error("Replay cache key is empty");
    return cleaned;
}

std::int64_t nowMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

std::filesystem::path indexPath() {
    return cacheDir() / "index.json";
}

std::filesystem::path blobPath(const std::string& key) {
    return cacheDir() / safeName(key);
}

std::filesystem::path partialPath(const std::string& key) {
    return cacheDir() / (safeName(key) + ".partial");
}

std::uint64_t freeBytes() {
    std::error_code error;
    const auto space = std::filesystem::space(cacheDir(), error);
    if(error) return 0;
    return static_cast<std::uint64_t>(space.available);
}

bool isScratchName(const std::string& name) {
    return name.rfind("pre_", 0) == 0 || name.rfind("unpack_", 0) == 0;
}

std::uint64_t fileBytes(const std::filesystem::path& path) {
    std::error_code error;
    if(!std::filesystem::is_regular_file(path, error) || error) return 0;
    const auto size = std::filesystem::file_size(path, error);
    if(error) return 0;
    return static_cast<std::uint64_t>(size);
}

std::uint64_t treeBytes(const std::filesystem::path& path) {
    std::error_code error;
    if(std::filesystem::is_regular_file(path, error) && !error) return fileBytes(path);
    if(error || !std::filesystem::is_directory(path)) return 0;
    std::uint64_t sum = 0;
    const auto options = std::filesystem::directory_options::skip_permission_denied;
    std::filesystem::recursive_directory_iterator cursor(path, options, error);
    const std::filesystem::recursive_directory_iterator end;
    while(!error && cursor != end) {
        sum += fileBytes(cursor->path());
        cursor.increment(error);
    }
    return sum;
}

struct ScratchDir {
    std::filesystem::path path;
    std::string name;
    std::uint64_t bytes = 0;
    std::filesystem::file_time_type mtime{};
};

std::vector<ScratchDir> scratchDirs() {
    std::vector<ScratchDir> dirs;
    std::error_code error;
    std::filesystem::directory_iterator cursor(cacheDir(), error);
    if(error) return dirs;
    const std::filesystem::directory_iterator end;
    for(; !error && cursor != end; cursor.increment(error)) {
        std::error_code entryError;
        if(!cursor->is_directory(entryError) || entryError) continue;
        const auto name = cursor->path().filename().string();
        if(!isScratchName(name)) continue;
        ScratchDir dir;
        dir.path = cursor->path();
        dir.name = name;
        dir.bytes = treeBytes(cursor->path());
        dir.mtime = std::filesystem::last_write_time(cursor->path(), entryError);
        dirs.push_back(std::move(dir));
    }
    return dirs;
}

std::string scratchHash(const std::string& name) {
    if(name.rfind("pre_", 0) == 0) return name.substr(4);
    if(name.rfind("unpack_", 0) == 0) return name.substr(7);
    return {};
}

bool scratchHeld(const std::string& name) {
    const auto lease = gScratchLeases.find(name);
    if(lease != gScratchLeases.end() && lease->second > 0) return true;
    const auto hash = scratchHash(name);
    if(hash.empty()) return false;
    for(const auto& entry : gBook.entries) {
        if(entry.armed <= 0) continue;
        if(entry.key == hash || entry.sourceHash == hash) return true;
    }
    return false;
}

std::uint64_t scratchBytes() {
    std::uint64_t sum = 0;
    for(const auto& dir : scratchDirs()) sum += dir.bytes;
    return sum;
}

std::uint64_t usedBytes() {
    std::uint64_t sum = 0;
    for(const auto& entry : gBook.entries) sum += entry.sizeBytes;
    sum += scratchBytes();
    return sum;
}

void throwFit(std::uint64_t needed) {
    throw std::runtime_error("Replay cache needs " + std::to_string(needed) + " bytes, limit " + std::to_string(gBook.limitBytes) + " bytes, free "
                             + std::to_string(freeBytes()) + " bytes");
}

Entry* findEntry(const std::string& key) {
    for(auto& entry : gBook.entries) {
        if(entry.key == key) return &entry;
    }
    return nullptr;
}

void saveIndex() {
    nlohmann::json rows = nlohmann::json::array();
    for(const auto& entry : gBook.entries) {
        rows.push_back({
            {"key", entry.key},
            {"kind", entry.kind},
            {"sourceHash", entry.sourceHash},
            {"sizeBytes", entry.sizeBytes},
            {"lastUseMs", entry.lastUseMs},
            {"armed", entry.armed},
            {"persistent", entry.persistent},
        });
    }
    nlohmann::json root = {{"limitBytes", gBook.limitBytes}, {"entries", std::move(rows)}};
    std::filesystem::create_directories(cacheDir());
    const auto temporary = indexPath().string() + ".tmp";
    {
        std::ofstream output;
        output.exceptions(std::ios::badbit | std::ios::failbit);
        output.open(temporary);
        output << root.dump();
        output.close();
    }
    std::filesystem::rename(temporary, indexPath());
}

void loadIndex() {
    if(gBook.loaded) return;
    gBook.loaded = true;
    std::filesystem::create_directories(cacheDir());
    for(const auto& file : std::filesystem::directory_iterator(cacheDir())) {
        if(file.path().extension() == ".partial") std::filesystem::remove_all(file.path());
    }
    std::ifstream input(indexPath());
    if(!input) return;
    nlohmann::json root;
    input >> root;
    gBook.limitBytes = root.value("limitBytes", kDefaultLimit);
    for(const auto& row : root.value("entries", nlohmann::json::array())) {
        Entry entry;
        entry.key = row.value("key", std::string{});
        entry.kind = row.value("kind", 0);
        entry.sourceHash = row.value("sourceHash", entry.key);
        entry.sizeBytes = row.value("sizeBytes", 0ull);
        entry.lastUseMs = row.value("lastUseMs", 0ll);
        // Holds belong to the previous firmware process, not to saved files.
        entry.armed = 0;
        entry.persistent = row.value("persistent", false);
        if(!entry.key.empty() && std::filesystem::exists(blobPath(entry.key))) gBook.entries.push_back(std::move(entry));
    }
}

void hashFileBytes(Sha256& hash, const std::filesystem::path& path) {
    std::ifstream input(path, std::ios::binary);
    if(!input) throw std::runtime_error("Replay cache upload cannot be read");
    std::vector<char> buffer(1 << 20);
    while(input) {
        input.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
        if(input.gcount() > 0) hash.update(reinterpret_cast<const std::uint8_t*>(buffer.data()), static_cast<std::size_t>(input.gcount()));
    }
    if(input.bad()) throw std::runtime_error("Replay cache upload read failed");
}

std::string hashFile(const std::filesystem::path& path) {
    Sha256 hash;
    hashFileBytes(hash, path);
    return hash.hex();
}

std::string hashDirectory(const std::filesystem::path& path, const std::map<std::string, std::uint64_t>& files) {
    Sha256 hash;
    const std::uint8_t zero = 0;
    for(const auto& file : files) {
        hash.update(reinterpret_cast<const std::uint8_t*>(file.first.data()), file.first.size());
        hash.update(&zero, 1);
        hashFileBytes(hash, path / file.first);
    }
    return hash.hex();
}

bool validFilename(const std::string& name) {
    return !name.empty() && name.front() != '.' && name.find_first_of("/\\") == std::string::npos && name.find('\0') == std::string::npos;
}

bool validHash(const std::string& hash) {
    return hash.size() == 64 && hash.find_first_not_of("0123456789abcdef") == std::string::npos;
}

Entry* sourceAtPath(const std::string& pathString) {
    if(pathString.empty()) return nullptr;
    const std::filesystem::path path(pathString);
    const auto key = path.filename().string();
    return validHash(key) && blobPath(key) == path ? findEntry(key) : nullptr;
}

void ensureSpace(std::uint64_t sizeBytes) {
    auto needed = sizeBytes;
    auto diskNeeded = sizeBytes;
    for(const auto& pending : gBook.pendingBytes) {
        if(pending.second > std::numeric_limits<std::uint64_t>::max() - needed) throwFit(std::numeric_limits<std::uint64_t>::max());
        needed += pending.second;
        const auto directory = gBook.pendingDirectories.find(pending.first);
        // Directory writes are ordered, so already-written bytes consume free
        // space and need no second disk reservation. File uploads may be sparse.
        if(directory != gBook.pendingDirectories.end()) diskNeeded += pending.second - directory->second.writtenBytes;
        else if(gBook.pendingPrepared.count(pending.first) != 0) diskNeeded += pending.second - gBook.pendingWritten.at(pending.first);
        else diskNeeded += pending.second;
    }
    if(needed > gBook.limitBytes) throwFit(needed);
    auto protectedBytes = needed;
    for(const auto& entry : gBook.entries) {
        if(entry.armed <= 0) continue;
        if(entry.sizeBytes > std::numeric_limits<std::uint64_t>::max() - protectedBytes) throwFit(std::numeric_limits<std::uint64_t>::max());
        protectedBytes += entry.sizeBytes;
    }
    // Evicting idle entries cannot help when the reservations and live sources
    // alone exceed the limit. Fail before removing any of those idle entries.
    for(const auto& dir : scratchDirs()) {
        if(!scratchHeld(dir.name)) continue;
        if(dir.bytes > std::numeric_limits<std::uint64_t>::max() - protectedBytes) throwFit(std::numeric_limits<std::uint64_t>::max());
        protectedBytes += dir.bytes;
    }
    if(protectedBytes > gBook.limitBytes) throwFit(protectedBytes);
    while(usedBytes() > gBook.limitBytes - needed || diskNeeded > freeBytes()) {
        const auto dirs = scratchDirs();
        const ScratchDir* scratchVictim = nullptr;
        for(const auto& dir : dirs) {
            if(scratchHeld(dir.name)) continue;
            if(scratchVictim == nullptr || dir.mtime < scratchVictim->mtime) scratchVictim = &dir;
        }
        if(scratchVictim != nullptr) {
            const auto path = scratchVictim->path;
            std::error_code error;
            std::filesystem::remove_all(path, error);
            if(std::filesystem::exists(path)) throwFit(needed);
            continue;
        }
        Entry* victim = nullptr;
        for(auto& entry : gBook.entries) {
            if(entry.armed > 0) continue;
            if(victim == nullptr || entry.lastUseMs < victim->lastUseMs || (entry.lastUseMs == victim->lastUseMs && entry.sizeBytes > victim->sizeBytes)) {
                victim = &entry;
            }
        }
        if(victim == nullptr) throwFit(needed);
        std::error_code error;
        std::filesystem::remove_all(blobPath(victim->key), error);
        const auto key = victim->key;
        gBook.entries.erase(std::remove_if(gBook.entries.begin(), gBook.entries.end(), [&](const Entry& entry) { return entry.key == key; }),
                            gBook.entries.end());
        saveIndex();
    }
}

void noteUpload(const std::string& hash, std::uint64_t done, std::uint64_t total) {
    if(total == 0) return;
    spdlog::info("Replay upload {} {}/{} bytes", hash, done, total);
}

std::tuple<bool, std::string, std::uint64_t> acquireSource(Entry& entry) {
    if(entry.armed == std::numeric_limits<int>::max() || entry.prepared == std::numeric_limits<int>::max()) {
        throw std::runtime_error("Replay cache hold count overflow");
    }
    const auto result = std::make_tuple(true, blobPath(entry.key).string(), entry.sizeBytes);
    const auto previous = entry;
    if(entry.armed == 0) entry.persistent = true;
    ++entry.armed;
    ++entry.prepared;
    entry.lastUseMs = nowMs();
    try {
        saveIndex();
    } catch(...) {
        entry = previous;
        throw;
    }
    return result;
}

void beginFileUpload(const std::string& hash, std::uint64_t totalBytes) {
    if(!validHash(hash)) throw std::runtime_error("Replay cache invalid source hash");
    if(findEntry(hash) != nullptr) throw std::runtime_error("Replay cache upload already exists");
    if(gBook.pendingBytes.count(hash) != 0) throw std::runtime_error("Replay cache upload already started");
    // A previous process can die after the final rename but before publishing
    // its index. Such an unindexed blob owns no live holds and is replaceable.
    std::filesystem::remove_all(blobPath(hash));
    ensureSpace(totalBytes);
    std::filesystem::create_directories(cacheDir());
    std::error_code error;
    std::filesystem::remove_all(partialPath(hash), error);
    {
        std::ofstream created(partialPath(hash), std::ios::binary | std::ios::trunc);
        if(!created) throw std::runtime_error("Replay cache upload cannot be created");
    }
    std::filesystem::resize_file(partialPath(hash), totalBytes);
    gBook.pendingBytes[hash] = totalBytes;
    gBook.pendingBucket[hash] = 0;
    gBook.pendingWritten[hash] = 0;
}

}  // namespace

std::tuple<std::uint64_t, std::uint64_t, std::vector<std::tuple<std::string, std::int32_t, std::string, std::uint64_t, std::int64_t, bool>>> replayCacheInfo() {
    std::lock_guard<std::mutex> lock(gMutex);
    loadIndex();
    std::vector<std::tuple<std::string, std::int32_t, std::string, std::uint64_t, std::int64_t, bool>> rows;
    rows.reserve(gBook.entries.size());
    for(const auto& entry : gBook.entries) {
        rows.emplace_back(entry.key, entry.kind, entry.sourceHash, entry.sizeBytes, entry.lastUseMs, entry.armed > 0);
    }
    return {gBook.limitBytes, usedBytes(), std::move(rows)};
}

void replayCacheSetLimit(std::uint64_t bytes) {
    std::lock_guard<std::mutex> lock(gMutex);
    loadIndex();
    const auto previous = gBook.limitBytes;
    gBook.limitBytes = bytes;
    try {
        saveIndex();
    } catch(...) {
        gBook.limitBytes = previous;
        throw;
    }
}

void replayCacheAcquireScratch(const std::string& name) {
    std::lock_guard<std::mutex> lock(gMutex);
    if(!isScratchName(name) || name.find('/') != std::string::npos || name.find('\\') != std::string::npos) {
        throw std::runtime_error("Replay cache scratch name is invalid");
    }
    auto& count = gScratchLeases[name];
    if(count == std::numeric_limits<int>::max()) throw std::runtime_error("Replay cache hold count overflow");
    ++count;
}

void replayCacheReleaseScratch(const std::string& name) {
    std::lock_guard<std::mutex> lock(gMutex);
    const auto found = gScratchLeases.find(name);
    if(found == gScratchLeases.end()) return;
    if(found->second > 0) --found->second;
    if(found->second <= 0) gScratchLeases.erase(found);
}

int replayCacheScratchHolders(const std::string& name) {
    std::lock_guard<std::mutex> lock(gMutex);
    const auto found = gScratchLeases.find(name);
    return found == gScratchLeases.end() ? 0 : found->second;
}

void replayCacheClear() {
    std::lock_guard<std::mutex> lock(gMutex);
    loadIndex();
    std::vector<Entry> kept;
    for(const auto& entry : gBook.entries) {
        if(entry.armed > 0) {
            kept.push_back(entry);
            continue;
        }
        std::error_code error;
        std::filesystem::remove_all(blobPath(entry.key), error);
    }
    for(const auto& dir : scratchDirs()) {
        if(scratchHeld(dir.name)) continue;
        std::error_code error;
        std::filesystem::remove_all(dir.path, error);
    }
    gBook.entries = std::move(kept);
    saveIndex();
}

std::tuple<bool, std::string, std::uint64_t> replayCacheQuery(const std::string& hash) {
    std::lock_guard<std::mutex> lock(gMutex);
    loadIndex();
    auto* entry = findEntry(hash);
    if(entry == nullptr || !std::filesystem::exists(blobPath(hash))) return {false, {}, 0};
    entry->lastUseMs = nowMs();
    saveIndex();
    return {true, blobPath(hash).string(), entry->sizeBytes};
}

void replayCacheBegin(const std::string& hash, std::uint64_t totalBytes) {
    std::lock_guard<std::mutex> lock(gMutex);
    loadIndex();
    beginFileUpload(hash, totalBytes);
}

std::tuple<bool, std::string, std::uint64_t> replayCachePrepareFile(const std::string& hash, std::uint64_t totalBytes) {
    std::lock_guard<std::mutex> lock(gMutex);
    loadIndex();
    if(!validHash(hash)) throw std::runtime_error("Replay cache invalid source hash");
    if(auto* entry = findEntry(hash)) {
        if(!std::filesystem::is_regular_file(blobPath(hash)) || entry->sizeBytes != totalBytes) throw std::runtime_error("Replay cache file differs");
        return acquireSource(*entry);
    }
    beginFileUpload(hash, totalBytes);
    gBook.pendingPrepared.insert(hash);
    return {false, blobPath(hash).string(), totalBytes};
}

std::tuple<bool, std::string, std::uint64_t> replayCacheBeginDirectory(const std::string& hash,
                                                                   const std::map<std::string, std::uint64_t>& files) {
    std::lock_guard<std::mutex> lock(gMutex);
    loadIndex();
    if(!validHash(hash)) throw std::runtime_error("Replay cache invalid directory hash");
    if(files.empty()) throw std::runtime_error("Replay cache directory manifest is empty");
    if(gBook.pendingBytes.count(hash) != 0) throw std::runtime_error("Replay cache upload already started");
    DirectoryUpload upload;
    upload.files = files;
    std::uint64_t total = 0;
    for(const auto& file : files) {
        if(!validFilename(file.first)) throw std::runtime_error("Replay cache invalid directory filename: " + file.first);
        if(file.second > std::numeric_limits<std::uint64_t>::max() - total) throw std::runtime_error("Replay cache directory size overflow");
        total += file.second;
        upload.written.emplace(file.first, 0);
    }
    if(auto* entry = findEntry(hash)) {
        if(!std::filesystem::is_directory(blobPath(hash)) || entry->sizeBytes != total) throw std::runtime_error("Replay cache directory manifest differs");
        return acquireSource(*entry);
    }
    std::filesystem::remove_all(blobPath(hash));
    ensureSpace(total);
    const auto partial = partialPath(hash);
    std::filesystem::remove_all(partial);
    try {
        std::filesystem::create_directories(partial);
        for(const auto& file : files) {
            std::ofstream output(partial / file.first, std::ios::binary | std::ios::trunc);
            if(!output) throw std::runtime_error("Replay cache directory file cannot be created: " + file.first);
        }
    } catch(...) {
        std::error_code error;
        std::filesystem::remove_all(partial, error);
        throw;
    }
    gBook.pendingBytes[hash] = total;
    gBook.pendingBucket[hash] = 0;
    gBook.pendingDirectories.emplace(hash, std::move(upload));
    return {false, blobPath(hash).string(), total};
}

void replayCacheWrite(const std::string& hash, std::uint64_t offset, const std::vector<std::uint8_t>& bytes) {
    std::lock_guard<std::mutex> lock(gMutex);
    loadIndex();
    const auto pending = gBook.pendingBytes.find(hash);
    if(pending == gBook.pendingBytes.end()) throw std::runtime_error("Replay cache upload was not started");
    if(gBook.pendingDirectories.count(hash) != 0) throw std::runtime_error("Replay cache directory upload requires a filename");
    if(offset > pending->second || bytes.size() > pending->second - offset) throw std::runtime_error("Replay cache upload exceeds the declared size");
    auto& written = gBook.pendingWritten.at(hash);
    if(gBook.pendingPrepared.count(hash) != 0 && offset != written) throw std::runtime_error("Replay cache file chunk is out of order");
    std::fstream output(partialPath(hash), std::ios::binary | std::ios::in | std::ios::out);
    if(!output) throw std::runtime_error("Replay cache upload cannot be written");
    output.seekp(static_cast<std::streamoff>(offset));
    output.write(reinterpret_cast<const char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
    output.close();
    if(!output) throw std::runtime_error("Replay cache upload write failed");
    const auto done = offset + static_cast<std::uint64_t>(bytes.size());
    written = std::max(written, done);
    const int bucket = pending->second == 0 ? 100 : static_cast<int>((done * 100) / pending->second);
    auto& lastBucket = gBook.pendingBucket[hash];
    if(bucket >= lastBucket + 10 || done == pending->second) {
        lastBucket = bucket;
        noteUpload(hash, done, pending->second);
    }
}

void replayCacheWriteFile(const std::string& hash, const std::string& filename, std::uint64_t offset, const std::vector<std::uint8_t>& bytes) {
    std::lock_guard<std::mutex> lock(gMutex);
    loadIndex();
    const auto pending = gBook.pendingDirectories.find(hash);
    if(pending == gBook.pendingDirectories.end()) throw std::runtime_error("Replay cache directory upload was not started");
    auto& upload = pending->second;
    const auto file = upload.files.find(filename);
    if(file == upload.files.end()) throw std::runtime_error("Replay cache directory file was not declared: " + filename);
    auto& written = upload.written.at(filename);
    if(offset != written || offset > file->second || bytes.size() > file->second - offset) {
        throw std::runtime_error("Replay cache directory chunk exceeds its file or is out of order");
    }
    std::fstream output(partialPath(hash) / filename, std::ios::binary | std::ios::in | std::ios::out);
    if(!output) throw std::runtime_error("Replay cache directory file cannot be written: " + filename);
    output.seekp(static_cast<std::streamoff>(offset));
    output.write(reinterpret_cast<const char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
    output.close();
    if(!output) throw std::runtime_error("Replay cache directory write failed: " + filename);
    written += static_cast<std::uint64_t>(bytes.size());
    upload.writtenBytes += static_cast<std::uint64_t>(bytes.size());
    const auto total = gBook.pendingBytes.at(hash);
    const auto done = upload.writtenBytes;
    const int bucket = total == 0 ? 100 : static_cast<int>(static_cast<long double>(done) * 100 / total);
    auto& lastBucket = gBook.pendingBucket.at(hash);
    if(bucket >= lastBucket + 10 || done == total) {
        lastBucket = bucket;
        noteUpload(hash, done, total);
    }
}

void replayCacheCommit(const std::string& hash) {
    std::lock_guard<std::mutex> lock(gMutex);
    loadIndex();
    const auto pending = gBook.pendingBytes.find(hash);
    if(pending == gBook.pendingBytes.end()) throw std::runtime_error("Replay cache upload was not started");
    const auto expected = pending->second;
    const auto partial = partialPath(hash);
    const auto clearPending = [&] {
        gBook.pendingBytes.erase(hash);
        gBook.pendingBucket.erase(hash);
        gBook.pendingDirectories.erase(hash);
        gBook.pendingPrepared.erase(hash);
        gBook.pendingWritten.erase(hash);
    };
    try {
        const auto directory = gBook.pendingDirectories.find(hash);
        std::string actual;
        if(directory != gBook.pendingDirectories.end()) {
            for(const auto& file : directory->second.files) {
                const auto path = partial / file.first;
                if(directory->second.written.at(file.first) != file.second || !std::filesystem::is_regular_file(std::filesystem::symlink_status(path))
                   || std::filesystem::file_size(path) != file.second) {
                    throw std::runtime_error("Replay cache directory upload is incomplete: " + file.first);
                }
            }
            actual = hashDirectory(partial, directory->second.files);
        } else {
            if(gBook.pendingPrepared.count(hash) != 0 && gBook.pendingWritten.at(hash) != expected) throw std::runtime_error("Replay cache upload is incomplete");
            if(std::filesystem::file_size(partial) != expected) throw std::runtime_error("Replay cache upload is incomplete");
            actual = hashFile(partial);
        }
        if(actual != hash) throw std::runtime_error("Replay cache hash mismatch: expected " + hash + ", got " + actual);
        std::filesystem::rename(partial, blobPath(hash));
        Entry entry;
        entry.key = hash;
        entry.sourceHash = hash;
        entry.sizeBytes = expected;
        entry.lastUseMs = nowMs();
        if(directory != gBook.pendingDirectories.end() || gBook.pendingPrepared.count(hash) != 0) {
            entry.armed = 1;
            entry.prepared = 1;
        }
        gBook.entries.push_back(std::move(entry));
        try {
            saveIndex();
        } catch(...) {
            gBook.entries.pop_back();
            std::error_code error;
            std::filesystem::remove_all(blobPath(hash), error);
            throw;
        }
        clearPending();
    } catch(...) {
        clearPending();
        std::error_code error;
        std::filesystem::remove_all(partial, error);
        throw;
    }
    spdlog::info("Replay cache upload {} {} bytes", hash, expected);
}

void replayCacheAbort(const std::string& hash) {
    std::lock_guard<std::mutex> lock(gMutex);
    loadIndex();
    gBook.pendingBytes.erase(hash);
    gBook.pendingBucket.erase(hash);
    gBook.pendingDirectories.erase(hash);
    gBook.pendingPrepared.erase(hash);
    gBook.pendingWritten.erase(hash);
    std::error_code error;
    std::filesystem::remove_all(partialPath(hash), error);
}

void replayCacheRelease(const std::vector<std::string>& keys) {
    std::lock_guard<std::mutex> lock(gMutex);
    loadIndex();
    for(const auto& key : keys) {
        auto* entry = findEntry(key);
        if(entry == nullptr) continue;
        if(entry->armed <= 0) continue;
        entry->armed -= 1;
        if(entry->prepared > 0) --entry->prepared;
        if(entry->armed > 0) continue;
        const bool remove = entry->started && !entry->persistent;
        entry->started = false;
        entry->prepared = 0;
        if(!remove) continue;
        std::error_code error;
        std::filesystem::remove_all(blobPath(entry->key), error);
        const auto drop = entry->key;
        gBook.entries.erase(std::remove_if(gBook.entries.begin(), gBook.entries.end(), [&](const Entry& item) { return item.key == drop; }),
                            gBook.entries.end());
    }
    saveIndex();
}

void replayCacheMarkPersistent(const std::string& hash, const std::string& metadataPath) {
    if(hash.empty() && metadataPath.empty()) return;
    std::lock_guard<std::mutex> lock(gMutex);
    loadIndex();
    const auto mark = [](Entry* entry) {
        if(entry == nullptr) return;
        entry->persistent = true;
        entry->lastUseMs = nowMs();
    };
    mark(findEntry(hash));
    if(auto* metadata = sourceAtPath(metadataPath); metadata != nullptr && metadata->key != hash) mark(metadata);
    saveIndex();
}

void replayCacheArm(const std::string& hash, const std::string& metadataPath) {
    if(hash.empty() && metadataPath.empty()) return;
    std::lock_guard<std::mutex> lock(gMutex);
    loadIndex();
    const auto arm = [](Entry* entry) {
        if(entry == nullptr) return;
        if(entry->prepared > 0) --entry->prepared;
        else {
            if(entry->armed == std::numeric_limits<int>::max()) throw std::runtime_error("Replay cache hold count overflow");
            ++entry->armed;
        }
        entry->started = true;
        entry->lastUseMs = nowMs();
    };
    arm(findEntry(hash));
    if(auto* metadata = sourceAtPath(metadataPath); metadata != nullptr && metadata->key != hash) arm(metadata);
    saveIndex();
}

}  // namespace gate
}  // namespace dai
