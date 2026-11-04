#include <algorithm>
#include <atomic>
#include <cctype>
#include <condition_variable>
#include <cstring>
#include <fstream>
#include <functional>
#include <mutex>
#include <nlohmann/json.hpp>
#include <set>
#include <stdexcept>

#include "ReplayInternal.hpp"
#include "utility/Environment.hpp"
#include "utility/Platform.hpp"

namespace dai {
namespace replay {
namespace {

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

    void update(const std::string& text) {
        update(reinterpret_cast<const std::uint8_t*>(text.data()), text.size());
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
            const auto S1 = rotr(e, 6) ^ rotr(e, 11) ^ rotr(e, 25);
            const auto ch = (e & f) ^ ((~e) & g);
            const auto temp1 = h + S1 + ch + k[i] + w[i];
            const auto S0 = rotr(a, 2) ^ rotr(a, 13) ^ rotr(a, 22);
            const auto maj = (a & b) ^ (a & c) ^ (b & c);
            const auto temp2 = S0 + maj;
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

struct Hold {
    std::uint64_t nodeId = 0;
    std::uint64_t token = 0;
};

struct Entry {
    std::string key;
    ReplayCacheKind kind = ReplayCacheKind::SOURCE;
    std::string sourceHash;
    std::uint64_t sizeBytes = 0;
    std::int64_t lastUseMs = 0;
    bool persistent = false;
    std::uint64_t createdToken = 0;
    bool ownsFiles = false;
    std::string relativePath;
    std::string externalPath;
    std::vector<Hold> holds;
};

struct Index {
    std::uint64_t limitBytes = kDefaultCacheLimit;
    std::vector<Entry> entries;
};

std::mutex gMutex;
std::condition_variable gCv;
std::set<std::string> gPending;
bool gLoaded = false;
std::atomic<bool> gStandalone{false};
std::filesystem::path gLoadedDir;
Index gIndex;

std::int64_t nowMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

std::filesystem::path indexPath(const std::filesystem::path& dir) {
    return dir / "index.json";
}

nlohmann::json holdJson(const Hold& hold) {
    return nlohmann::json{{"node", hold.nodeId}, {"token", hold.token}};
}

nlohmann::json entryJson(const Entry& entry) {
    nlohmann::json holds = nlohmann::json::array();
    for(const auto& hold : entry.holds) holds.push_back(holdJson(hold));
    return nlohmann::json{{"key", entry.key},
                          {"kind", static_cast<int>(entry.kind)},
                          {"sourceHash", entry.sourceHash},
                          {"sizeBytes", entry.sizeBytes},
                          {"lastUseMs", entry.lastUseMs},
                          {"persistent", entry.persistent},
                          {"createdToken", entry.createdToken},
                          {"ownsFiles", entry.ownsFiles},
                          {"relativePath", entry.relativePath},
                          {"externalPath", entry.externalPath},
                          {"holds", std::move(holds)}};
}

Entry entryFromJson(const nlohmann::json& json) {
    Entry entry;
    entry.key = json.value("key", "");
    entry.kind = static_cast<ReplayCacheKind>(json.value("kind", 0));
    entry.sourceHash = json.value("sourceHash", "");
    entry.sizeBytes = json.value("sizeBytes", 0ull);
    entry.lastUseMs = json.value("lastUseMs", 0ll);
    entry.persistent = json.value("persistent", false);
    entry.createdToken = json.value("createdToken", 0ull);
    entry.ownsFiles = json.value("ownsFiles", false);
    entry.relativePath = json.value("relativePath", "");
    entry.externalPath = json.value("externalPath", "");
    if(json.contains("holds")) {
        for(const auto& hold : json["holds"]) {
            entry.holds.push_back(Hold{hold.value("node", 0ull), hold.value("token", 0ull)});
        }
    }
    return entry;
}

void saveIndex(const std::filesystem::path& dir) {
    nlohmann::json entries = nlohmann::json::array();
    for(const auto& entry : gIndex.entries) entries.push_back(entryJson(entry));
    nlohmann::json root{{"limitBytes", gIndex.limitBytes}, {"entries", std::move(entries)}};
    std::filesystem::create_directories(dir);
    const auto path = indexPath(dir);
    const auto temporary = path.string() + ".tmp";
    {
        std::ofstream out(temporary, std::ios::trunc);
        if(!out) throw std::runtime_error("Replay cache index cannot be written: " + path.string());
        out << root.dump(2);
    }
    std::filesystem::rename(temporary, path);
}

void loadIndex(const std::filesystem::path& dir) {
    if(gLoaded && gLoadedDir == dir) return;
    gIndex = Index{};
    gLoaded = true;
    gLoadedDir = dir;
    const auto path = indexPath(dir);
    if(!std::filesystem::exists(path)) return;
    std::ifstream in(path);
    if(!in) return;
    nlohmann::json root;
    try {
        in >> root;
    } catch(const std::exception&) {
        return;
    }
    gIndex.limitBytes = root.value("limitBytes", kDefaultCacheLimit);
    if(root.contains("entries")) {
        for(const auto& item : root["entries"]) gIndex.entries.push_back(entryFromJson(item));
    }
}

Entry* findEntry(const std::string& key) {
    for(auto& entry : gIndex.entries) {
        if(entry.key == key) return &entry;
    }
    return nullptr;
}

std::uint64_t usedBytes() {
    std::uint64_t total = 0;
    for(const auto& entry : gIndex.entries) total += entry.sizeBytes;
    return total;
}

struct Unit {
    std::string hash;
    std::uint64_t size = 0;
    std::int64_t lastUse = 0;
    bool inUse = false;
};

std::vector<Unit> unitsOf() {
    std::vector<Unit> units;
    for(const auto& entry : gIndex.entries) {
        Unit* unit = nullptr;
        for(auto& candidate : units) {
            if(candidate.hash == entry.sourceHash) {
                unit = &candidate;
                break;
            }
        }
        if(unit == nullptr) {
            units.push_back(Unit{});
            unit = &units.back();
            unit->hash = entry.sourceHash;
            unit->lastUse = entry.lastUseMs;
        }
        unit->size += entry.sizeBytes;
        unit->lastUse = std::max(unit->lastUse, entry.lastUseMs);
        if(!entry.holds.empty()) unit->inUse = true;
    }
    return units;
}

void removeUnitFiles(const std::filesystem::path& dir, const std::string& hash) {
    for(const auto& entry : gIndex.entries) {
        if(entry.sourceHash != hash || !entry.ownsFiles || entry.relativePath.empty()) continue;
        std::error_code error;
        std::filesystem::remove_all(dir / entry.relativePath, error);
    }
}

void eraseUnit(const std::string& hash) {
    gIndex.entries.erase(std::remove_if(gIndex.entries.begin(), gIndex.entries.end(), [&](const Entry& entry) { return entry.sourceHash == hash; }),
                         gIndex.entries.end());
}

std::uint64_t freeBytes(const std::filesystem::path& dir) {
    std::error_code error;
    const auto space = std::filesystem::space(dir, error);
    if(error) return 0;
    return static_cast<std::uint64_t>(space.available);
}

void throwFit(std::uint64_t needed, std::uint64_t limit, std::uint64_t freeSpace) {
    throw std::runtime_error("Replay cache needs " + std::to_string(needed) + " bytes, limit " + std::to_string(limit) + " bytes, free "
                             + std::to_string(freeSpace) + " bytes");
}

void ensureSpace(const std::filesystem::path& dir, const CacheSlotRequest& request) {
    if(findEntry(request.key) != nullptr) return;
    const auto limit = gIndex.limitBytes;
    if(request.sizeBytes > limit) throwFit(request.sizeBytes, limit, freeBytes(dir));
    auto fits = [&]() { return usedBytes() + request.sizeBytes <= limit && request.diskBytes <= freeBytes(dir); };
    while(!fits()) {
        auto groups = unitsOf();
        std::vector<Unit> victims;
        for(const auto& unit : groups) {
            if(unit.inUse || unit.hash == request.sourceHash) continue;
            victims.push_back(unit);
        }
        if(victims.empty()) {
            saveIndex(dir);
            throwFit(std::max(request.sizeBytes, request.diskBytes), limit, freeBytes(dir));
        }
        std::sort(victims.begin(), victims.end(), [](const Unit& a, const Unit& b) {
            if(a.lastUse != b.lastUse) return a.lastUse < b.lastUse;
            return a.size > b.size;
        });
        removeUnitFiles(dir, victims.front().hash);
        eraseUnit(victims.front().hash);
        saveIndex(dir);
    }
}

void addHold(Entry& entry, std::uint64_t nodeId, std::uint64_t token, bool persistent) {
    for(const auto& hold : entry.holds) {
        if(hold.nodeId == nodeId) {
            entry.lastUseMs = nowMs();
            if(persistent) entry.persistent = true;
            return;
        }
    }
    entry.holds.push_back(Hold{nodeId, token});
    entry.lastUseMs = nowMs();
    if(persistent) entry.persistent = true;
}

void dropHolds(const std::filesystem::path& dir, const std::function<bool(const Hold&)>& match, std::uint64_t tokenFilter) {
    for(auto& entry : gIndex.entries) {
        entry.holds.erase(std::remove_if(entry.holds.begin(), entry.holds.end(), match), entry.holds.end());
    }
    std::vector<std::string> removeKeys;
    for(const auto& entry : gIndex.entries) {
        if(!entry.holds.empty() || entry.persistent || entry.createdToken != tokenFilter) continue;
        if(entry.ownsFiles && !entry.relativePath.empty()) {
            std::error_code error;
            std::filesystem::remove_all(dir / entry.relativePath, error);
        }
        removeKeys.push_back(entry.key);
    }
    gIndex.entries.erase(std::remove_if(gIndex.entries.begin(),
                                        gIndex.entries.end(),
                                        [&](const Entry& entry) { return std::find(removeKeys.begin(), removeKeys.end(), entry.key) != removeKeys.end(); }),
                         gIndex.entries.end());
}

std::filesystem::path safeRelative(const std::string& key) {
    std::string cleaned;
    cleaned.reserve(key.size());
    for(const char character : key) {
        const auto asUnsigned = static_cast<unsigned char>(character);
        if(std::isalnum(asUnsigned) || character == '_' || character == '-' || character == '.')
            cleaned.push_back(character);
        else
            cleaned.push_back('_');
    }
    return cleaned;
}

}  // namespace

void setReplayCacheStandalone(bool standalone) {
    gStandalone.store(standalone);
}

std::filesystem::path hostCacheDirectory(bool standalone) {
    const auto overrideDir = utility::getEnvAs<std::string>("DEPTHAI_REPLAY_CACHE", "", false);
    if(!overrideDir.empty()) return overrideDir;
    if(standalone || gStandalone.load()) return std::filesystem::path("/data/depthai/replay_cache");
    return platform::getTempPath() / "depthai_replay_cache";
}

std::string sha256OfBytes(const std::uint8_t* data, std::size_t size) {
    Sha256 hash;
    hash.update(data, size);
    return hash.hex();
}

std::string sha256OfFile(const std::filesystem::path& path) {
    std::ifstream in(path, std::ios::binary);
    if(!in) throw std::runtime_error("ReplayVideo source cannot be read: " + path.string());
    Sha256 hash;
    std::vector<char> buffer(1 << 20);
    while(in) {
        in.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
        if(in.gcount() > 0) hash.update(reinterpret_cast<const std::uint8_t*>(buffer.data()), static_cast<std::size_t>(in.gcount()));
    }
    return hash.hex();
}

std::string sha256OfDirectory(const std::filesystem::path& directory) {
    std::vector<std::filesystem::path> files;
    for(const auto& entry : std::filesystem::directory_iterator(directory)) {
        if(!entry.is_regular_file()) continue;
        const auto name = entry.path().filename().string();
        if(name.empty() || name.front() == '.') continue;
        files.push_back(entry.path());
    }
    std::sort(files.begin(), files.end(), [](const std::filesystem::path& a, const std::filesystem::path& b) {
        return a.filename().string() < b.filename().string();
    });
    Sha256 hash;
    for(const auto& file : files) {
        const auto name = file.filename().string();
        hash.update(name);
        hash.update(reinterpret_cast<const std::uint8_t*>("\0"), 1);
        std::ifstream in(file, std::ios::binary);
        if(!in) throw std::runtime_error("ReplayVideo source cannot be read: " + file.string());
        std::vector<char> buffer(1 << 20);
        while(in) {
            in.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
            if(in.gcount() > 0) hash.update(reinterpret_cast<const std::uint8_t*>(buffer.data()), static_cast<std::size_t>(in.gcount()));
        }
    }
    return hash.hex();
}

std::string sha256OfFileAndMetadata(const std::filesystem::path& file, const std::filesystem::path& metadata) {
    Sha256 hash;
    const auto fileName = file.filename().string();
    hash.update(fileName);
    hash.update(reinterpret_cast<const std::uint8_t*>("\0"), 1);
    {
        std::ifstream in(file, std::ios::binary);
        if(!in) throw std::runtime_error("ReplayVideo source cannot be read: " + file.string());
        std::vector<char> buffer(1 << 20);
        while(in) {
            in.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
            if(in.gcount() > 0) hash.update(reinterpret_cast<const std::uint8_t*>(buffer.data()), static_cast<std::size_t>(in.gcount()));
        }
    }
    const auto metaName = metadata.filename().string();
    hash.update(metaName);
    hash.update(reinterpret_cast<const std::uint8_t*>("\0"), 1);
    std::ifstream in(metadata, std::ios::binary);
    if(!in) throw std::runtime_error("ReplayVideo metadata file is missing: " + metadata.string());
    std::vector<char> buffer(1 << 20);
    while(in) {
        in.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
        if(in.gcount() > 0) hash.update(reinterpret_cast<const std::uint8_t*>(buffer.data()), static_cast<std::size_t>(in.gcount()));
    }
    return hash.hex();
}

CacheSlot beginCacheEntry(std::uint64_t nodeId, std::uint64_t pipelineToken, bool persistent, const CacheSlotRequest& request) {
    const auto dir = hostCacheDirectory(false);
    std::unique_lock<std::mutex> lock(gMutex);
    std::filesystem::create_directories(dir);
    loadIndex(dir);
    gCv.wait(lock, [&]() { return gPending.count(request.key) == 0; });
    CacheSlot slot;
    if(auto* existing = findEntry(request.key)) {
        addHold(*existing, nodeId, pipelineToken, persistent);
        saveIndex(dir);
        slot.hit = true;
        if(existing->ownsFiles && !existing->relativePath.empty())
            slot.path = dir / existing->relativePath;
        else
            slot.path = existing->externalPath;
        if(existing->kind == ReplayCacheKind::SOURCE)
            slot.logs.push_back("Replay cache hit " + existing->sourceHash);
        else
            slot.logs.push_back("Replay cache hit " + existing->sourceHash);
        return slot;
    }
    ensureSpace(dir, request);
    gPending.insert(request.key);
    saveIndex(dir);
    slot.hit = false;
    if(request.ownsFiles) {
        slot.path = dir / safeRelative(request.key);
        std::filesystem::create_directories(slot.path);
    } else {
        slot.path = request.externalPath;
    }
    return slot;
}

void commitCacheEntry(std::uint64_t nodeId, std::uint64_t pipelineToken, bool persistent, const CacheSlotRequest& request) {
    const auto dir = hostCacheDirectory(false);
    std::lock_guard<std::mutex> lock(gMutex);
    loadIndex(dir);
    gPending.erase(request.key);
    if(auto* existing = findEntry(request.key)) {
        addHold(*existing, nodeId, pipelineToken, persistent);
    } else {
        Entry entry;
        entry.key = request.key;
        entry.kind = request.kind;
        entry.sourceHash = request.sourceHash;
        entry.sizeBytes = request.sizeBytes;
        entry.lastUseMs = nowMs();
        entry.persistent = persistent;
        entry.createdToken = pipelineToken;
        entry.ownsFiles = request.ownsFiles;
        entry.relativePath = request.ownsFiles ? safeRelative(request.key).string() : "";
        entry.externalPath = request.ownsFiles ? "" : request.externalPath.string();
        entry.holds.push_back(Hold{nodeId, pipelineToken});
        gIndex.entries.push_back(std::move(entry));
    }
    saveIndex(dir);
    gCv.notify_all();
}

void abortCacheEntry(const std::string& key) {
    const auto dir = hostCacheDirectory(false);
    std::lock_guard<std::mutex> lock(gMutex);
    gPending.erase(key);
    std::error_code error;
    std::filesystem::remove_all(dir / safeRelative(key), error);
    gCv.notify_all();
}

void releaseReplayNode(std::uint64_t nodeId, std::uint64_t pipelineToken) {
    const auto dir = hostCacheDirectory(false);
    std::lock_guard<std::mutex> lock(gMutex);
    if(!std::filesystem::exists(dir)) return;
    loadIndex(dir);
    dropHolds(dir, [&](const Hold& hold) { return hold.nodeId == nodeId; }, pipelineToken);
    saveIndex(dir);
}

void releaseReplayToken(std::uint64_t pipelineToken) {
    const auto dir = hostCacheDirectory(false);
    std::lock_guard<std::mutex> lock(gMutex);
    if(!std::filesystem::exists(dir)) return;
    loadIndex(dir);
    dropHolds(dir, [&](const Hold& hold) { return hold.token == pipelineToken; }, pipelineToken);
    saveIndex(dir);
}

ReplayCacheInfo readHostCacheInfo() {
    const auto dir = hostCacheDirectory(false);
    std::lock_guard<std::mutex> lock(gMutex);
    loadIndex(dir);
    ReplayCacheInfo info;
    info.limitBytes = gIndex.limitBytes;
    for(const auto& entry : gIndex.entries) {
        ReplayCacheEntry row;
        row.key = entry.key;
        row.kind = entry.kind;
        row.sourceHash = entry.sourceHash;
        row.sizeBytes = entry.sizeBytes;
        row.lastUseMs = entry.lastUseMs;
        row.inUse = !entry.holds.empty();
        info.usedBytes += entry.sizeBytes;
        info.entries.push_back(std::move(row));
    }
    return info;
}

void writeHostCacheLimit(std::uint64_t bytes) {
    const auto dir = hostCacheDirectory(false);
    std::lock_guard<std::mutex> lock(gMutex);
    std::filesystem::create_directories(dir);
    loadIndex(dir);
    gIndex.limitBytes = bytes;
    saveIndex(dir);
}

void clearHostCache() {
    const auto dir = hostCacheDirectory(false);
    std::lock_guard<std::mutex> lock(gMutex);
    if(!std::filesystem::exists(dir)) return;
    loadIndex(dir);
    std::vector<std::string> hashes;
    for(const auto& entry : gIndex.entries) {
        if(entry.holds.empty()) hashes.push_back(entry.sourceHash);
    }
    std::sort(hashes.begin(), hashes.end());
    hashes.erase(std::unique(hashes.begin(), hashes.end()), hashes.end());
    for(const auto& hash : hashes) {
        bool busy = false;
        for(const auto& entry : gIndex.entries) {
            if(entry.sourceHash == hash && !entry.holds.empty()) busy = true;
        }
        if(busy) continue;
        removeUnitFiles(dir, hash);
        eraseUnit(hash);
    }
    saveIndex(dir);
}

}  // namespace replay
}  // namespace dai
