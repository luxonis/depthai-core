// RVC4 camera EEPROM access:
// - the camera module (CCM) EEPROM through readCcmEepromRaw() / writeCcmEepromRaw(), including cameras sharing one I2C bus
//   (CAM_B and CAM_C on OAK4-D), where the firmware allows access only while the camera is the only active one on its bus
// - the camera board (CBA) EEPROM through the CBA calibration API, read only (see its test case)
// The test controls which cameras stream. Every write is restored, and EEPROM contents never reach the log: only
// fingerprints, offsets and counts do.
#include <algorithm>
#include <atomic>
#include <catch2/catch_all.hpp>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <iterator>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include "depthai/depthai.hpp"
#include "depthai/device/EepromError.hpp"

using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;
using dai::CameraBoardSocket;

namespace {

constexpr auto CAM_A = CameraBoardSocket::CAM_A;
constexpr auto CAM_B = CameraBoardSocket::CAM_B;
constexpr auto CAM_C = CameraBoardSocket::CAM_C;

// The CCM EEPROM size the firmware exposes, and the window the write cases overwrite and restore
constexpr int CCM_EEPROM_SIZE = 8192;
constexpr int WRITE_WINDOW = 20;
constexpr int RESTORE_ATTEMPTS = 3;
// Two readouts must differ in at least this many programmed bytes before one counts as the bitwise AND of both,
// so modules that differ only in a few bytes (e.g. a serial number) aren't mistaken for one
constexpr size_t MIN_AND_DIFFERENCES = 8;
constexpr auto FIRST_FRAME_TIMEOUT = 15s;
constexpr auto FRAME_PROGRESS_TIMEOUT = 2s;

// Parts of the firmware's CCM EEPROM errors that the cases expect
constexpr const char* SHARED_BUS_ERROR = "must be the only active camera on it";
constexpr const char* NOT_POWERED_ERROR = "the camera must be streaming";
constexpr const char* NOTHING_WRITTEN_ERROR = "write failed at offset 0:";

std::string name(CameraBoardSocket socket) {
    return dai::toString(socket);
}

void report(const std::string& line) {
    std::printf("%s\n", line.c_str());
    std::fflush(stdout);
}

std::string join(const std::vector<std::string>& lines) {
    std::string text;
    for(const auto& line : lines) text += (text.empty() ? "" : "; ") + line;
    return text;
}

bool contains(const std::string& text, const std::string& part) {
    return text.find(part) != std::string::npos;
}

// FNV-1a, so the log identifies contents without showing them
std::string fingerprint(const std::vector<uint8_t>& data) {
    uint64_t hash = 1469598103934665603ull;
    for(const auto byte : data) hash = (hash ^ byte) * 1099511628211ull;
    char text[17];
    std::snprintf(text, sizeof(text), "%016llx", static_cast<unsigned long long>(hash));
    return text;
}

std::string fingerprint(const std::string& text) {
    return fingerprint(std::vector<uint8_t>(text.begin(), text.end()));
}

bool isBlank(const std::vector<uint8_t>& data) {
    return std::all_of(data.begin(), data.end(), [](uint8_t byte) { return byte == 0xFF; })
           || std::all_of(data.begin(), data.end(), [](uint8_t byte) { return byte == 0x00; });
}

// Where two buffers differ, without showing their contents
std::string difference(const std::vector<uint8_t>& got, const std::vector<uint8_t>& expected) {
    if(got.size() != expected.size()) return "got " + std::to_string(got.size()) + " bytes, expected " + std::to_string(expected.size());
    size_t count = 0;
    size_t first = 0;
    for(size_t i = 0; i < got.size(); ++i) {
        if(got[i] != expected[i] && count++ == 0) first = i;
    }
    if(count == 0) return "identical";
    return std::to_string(count) + " bytes differ, the first at offset " + std::to_string(first);
}

// Two modules answering at once on the open-drain bus read as the bitwise AND of both EEPROMs. Offsets where the
// other EEPROM is blank are left out, since a module with more data than another legitimately looks like that there
bool readsAsAndOf(const std::vector<uint8_t>& mixed, const std::vector<uint8_t>& other) {
    size_t differences = 0;
    for(size_t i = 0; i < mixed.size(); ++i) {
        if((mixed[i] & other[i]) != mixed[i]) return false;
        if(mixed[i] != other[i] && other[i] != 0xFF) ++differences;
    }
    return differences >= MIN_AND_DIFFERENCES;
}

// Random printable bytes, so never 0x00 or 0xFF, each differing from the backup and from the other payloads at its
// offset, so that a read-back can't match by accident
std::vector<uint8_t> makePayload(const std::vector<uint8_t>& backup, const std::vector<std::vector<uint8_t>>& others) {
    static std::mt19937 rng = [] {
        const auto seed = std::random_device{}();
        report("Payload seed " + std::to_string(seed));
        return std::mt19937(seed);
    }();
    std::uniform_int_distribution<int> printable(0x21, 0x7E);
    std::vector<uint8_t> payload(backup.size());
    for(size_t i = 0; i < payload.size(); ++i) {
        uint8_t byte = 0;
        do {
            byte = static_cast<uint8_t>(printable(rng));
        } while(byte == backup[i] || std::any_of(others.begin(), others.end(), [&](const std::vector<uint8_t>& other) { return other[i] == byte; }));
        payload[i] = byte;
    }
    return payload;
}

// Outcome of one EEPROM call
struct Outcome {
    bool ok = false;
    std::string error;
    std::vector<uint8_t> data;
};

std::string describe(const Outcome& outcome) {
    return outcome.ok ? "OK" : outcome.error;
}

bool refused(const Outcome& outcome) {
    return !outcome.ok && contains(outcome.error, SHARED_BUS_ERROR);
}

bool notPowered(const Outcome& outcome) {
    return !outcome.ok && contains(outcome.error, NOT_POWERED_ERROR);
}

// Counts the outcomes of repeated calls
std::string summarize(const std::vector<Outcome>& outcomes) {
    size_t ok = 0;
    size_t refusals = 0;
    size_t others = 0;
    std::string otherError;
    for(const auto& outcome : outcomes) {
        if(outcome.ok) {
            ++ok;
        } else if(refused(outcome)) {
            ++refusals;
        } else if(others++ == 0) {
            otherError = outcome.error;
        }
    }
    return std::to_string(outcomes.size()) + " calls: " + std::to_string(ok) + " OK, " + std::to_string(refusals) + " refused, " + std::to_string(others)
           + " other errors" + (otherError.empty() ? "" : " (" + otherError + ")");
}

Outcome readCcm(dai::Device& device, CameraBoardSocket socket, int size) {
    try {
        return {true, "", device.readCcmEepromRaw(socket, size, 0)};
    } catch(const std::exception& ex) {
        return {false, ex.what(), {}};
    }
}

Outcome writeCcm(dai::Device& device, CameraBoardSocket socket, const std::vector<uint8_t>& data) {
    try {
        device.writeCcmEepromRaw(socket, data, 0);
        return {true, "", {}};
    } catch(const std::exception& ex) {
        return {false, ex.what(), {}};
    }
}

// A device connection streaming the given cameras. The test owns the streams: it starts them, waits for their frames
// before touching an EEPROM and checks they keep running. Each session is a new connection, so nothing a previous
// case started is still running
class Session {
   public:
    explicit Session(const std::vector<CameraBoardSocket>& sockets) : device(std::make_shared<dai::Device>()), pipeline(device) {
        // Only the requested cameras: no calibration node added for a stereo pair
        pipeline.setAutoCalibrationMode(dai::PipelineAutoCalibrationMode::OFF);
        for(const auto socket : sockets) {
            queues.push_back(pipeline.create<dai::node::Camera>()->build(socket)->requestOutput({640, 400})->createOutputQueue(4, false));
        }
        frames.resize(queues.size(), 0);
    }

    ~Session() {
        counting = false;
        if(counter.joinable()) counter.join();
    }

    Session(const Session&) = delete;
    Session& operator=(const Session&) = delete;

    dai::Device& dev() {
        return *device;
    }

    void start() {
        pipeline.start();
        counter = std::thread([this] {
            try {
                while(counting) {
                    for(size_t i = 0; i < queues.size(); ++i) {
                        const auto count = queues[i]->tryGetAll<dai::ImgFrame>().size();
                        std::lock_guard<std::mutex> lock(framesMtx);
                        frames[i] += count;
                    }
                    std::this_thread::sleep_for(5ms);
                }
            } catch(const dai::MessageQueue::QueueException&) {
                // The pipeline stopped: the counts stop growing, which streaming() reports
            }
        });
    }

    // Every camera delivered a frame, so its module is powered
    bool firstFrames() {
        return framesAfter(std::vector<uint64_t>(queues.size(), 0), FIRST_FRAME_TIMEOUT);
    }

    // Every camera is still delivering frames
    bool streaming() {
        return framesAfter(frameCounts(), FRAME_PROGRESS_TIMEOUT);
    }

   private:
    std::vector<uint64_t> frameCounts() {
        std::lock_guard<std::mutex> lock(framesMtx);
        return frames;
    }

    bool framesAfter(const std::vector<uint64_t>& since, std::chrono::milliseconds timeout) {
        const auto deadline = Clock::now() + timeout;
        while(true) {
            const auto counts = frameCounts();
            bool all = true;
            for(size_t i = 0; i < counts.size(); ++i) all = all && counts[i] > since[i];
            if(all) return true;
            if(Clock::now() >= deadline) return false;
            std::this_thread::sleep_for(10ms);
        }
    }

    std::shared_ptr<dai::Device> device;
    dai::Pipeline pipeline;
    std::vector<std::shared_ptr<dai::MessageQueue>> queues;
    std::mutex framesMtx;
    std::vector<uint64_t> frames;
    std::atomic<bool> counting{true};
    std::thread counter;
};

// The CCM EEPROM of every connected camera, read in full while that camera streams alone, before any case writes.
// Distinct modules must read distinct contents: identical readouts mean two sockets reach the same EEPROM, or a
// module that should be off answers on a shared bus as well
struct CcmInventory {
    dai::Platform platform{};
    std::vector<CameraBoardSocket> connected;
    std::map<CameraBoardSocket, std::vector<uint8_t>> eeproms;
    std::vector<std::string> problems;
};

// The size an EEPROM smaller than the readout wraps around at: reading past its end starts over at offset 0
size_t wrapSize(const std::vector<uint8_t>& data) {
    for(size_t size = 64; size < data.size(); size *= 2) {
        bool wraps = true;
        for(size_t i = size; i < data.size() && wraps; ++i) wraps = data[i] == data[i % size];
        if(wraps) return size;
    }
    return data.size();
}

// Reads the CCM EEPROM of one camera, streaming alone, into the inventory
void readAlone(CameraBoardSocket socket, CcmInventory& inventory) {
    Session session({socket});
    session.start();
    if(!session.firstFrames()) {
        inventory.problems.push_back(name(socket) + " delivered no frames");
        return;
    }
    std::this_thread::sleep_for(1s);
    const auto first = readCcm(session.dev(), socket, CCM_EEPROM_SIZE);
    if(notPowered(first)) {
        // The camera streams, so its module is powered: it has no EEPROM
        report("CCM EEPROM " + name(socket) + ": not detected");
        return;
    }
    const auto second = readCcm(session.dev(), socket, CCM_EEPROM_SIZE);
    if(!first.ok || !second.ok) {
        inventory.problems.push_back(name(socket) + " read failed: " + describe(first.ok ? second : first));
    } else if(first.data != second.data) {
        inventory.problems.push_back(name(socket) + " read differently twice: " + difference(second.data, first.data));
    } else {
        report("CCM EEPROM " + name(socket) + ": detected, " + (isBlank(first.data) ? "blank" : std::to_string(wrapSize(first.data)) + " bytes")
               + ", fingerprint " + fingerprint(first.data));
        inventory.eeproms[socket] = first.data;
    }
}

void checkUnique(CcmInventory& inventory) {
    for(auto a = inventory.eeproms.begin(); a != inventory.eeproms.end(); ++a) {
        for(auto b = std::next(a); b != inventory.eeproms.end(); ++b) {
            const auto pair = name(a->first) + " and " + name(b->first);
            if(a->second == b->second) {
                inventory.problems.push_back(pair + " read identical contents: they reach the same EEPROM, or both modules answer at once");
            } else if(readsAsAndOf(a->second, b->second) || readsAsAndOf(b->second, a->second)) {
                inventory.problems.push_back(pair + ": one reads as the bitwise AND of both EEPROMs, so a module that should be off answers too");
            }
        }
    }
}

const CcmInventory& ccmInventory() {
    static const CcmInventory inventory = [] {
        CcmInventory result;
        {
            dai::Device device;
            result.platform = device.getPlatform();
            if(result.platform != dai::Platform::RVC4) return result;
            for(const auto& features : device.getConnectedCameraFeatures()) result.connected.push_back(features.socket);
        }
        for(const auto socket : result.connected) readAlone(socket, result);
        checkUnique(result);
        for(const auto& problem : result.problems) report("CCM EEPROM problem: " + problem);
        return result;
    }();
    return inventory;
}

void requireConnected(const std::vector<CameraBoardSocket>& sockets) {
    const auto& inventory = ccmInventory();
    if(inventory.platform != dai::Platform::RVC4) SKIP("CCM EEPROM access is tested on RVC4 only");
    for(const auto socket : sockets) {
        if(std::find(inventory.connected.begin(), inventory.connected.end(), socket) == inventory.connected.end()) SKIP(name(socket) << " is not connected");
    }
}

// Writing needs readouts the inventory could trust: otherwise a restore could put one EEPROM's contents into another
void requireCcmEeproms(const std::vector<CameraBoardSocket>& sockets) {
    requireConnected(sockets);
    const auto& inventory = ccmInventory();
    INFO(join(inventory.problems));
    REQUIRE(inventory.problems.empty());
    for(const auto socket : sockets) {
        if(inventory.eeproms.count(socket) == 0) SKIP("No CCM EEPROM detected on " << name(socket));
    }
}

std::vector<uint8_t> inventoryWindow(CameraBoardSocket socket) {
    const auto& eeprom = ccmInventory().eeproms.at(socket);
    return {eeprom.begin(), eeprom.begin() + WRITE_WINDOW};
}

// Writes the backup back and reads it to confirm, retrying a few times. A window that still holds the backup isn't
// written at all, so an EEPROM that rejected the write isn't sent more writes it would NACK
bool restoreWindow(dai::Device& device, CameraBoardSocket socket, const std::vector<uint8_t>& backup) {
    const auto current = readCcm(device, socket, static_cast<int>(backup.size()));
    if(current.ok && current.data == backup) {
        report("CCM EEPROM " + name(socket) + ": unchanged, nothing to restore");
        return true;
    }
    for(int attempt = 1; attempt <= RESTORE_ATTEMPTS; ++attempt) {
        const auto write = writeCcm(device, socket, backup);
        const auto check = readCcm(device, socket, static_cast<int>(backup.size()));
        if(write.ok && check.ok && check.data == backup) return true;
        report("CCM EEPROM " + name(socket) + " restore attempt " + std::to_string(attempt) + " failed: "
               + (!write.ok   ? write.error
                  : !check.ok ? check.error
                              : difference(check.data, backup)));
    }
    return false;
}

// CCM EEPROM windows written but not restored yet. restore() writes each one back in a new session that streams its
// camera alone. The destructor does the same when a case ends early
class PendingRestores {
   public:
    PendingRestores() = default;
    PendingRestores(const PendingRestores&) = delete;
    PendingRestores& operator=(const PendingRestores&) = delete;

    ~PendingRestores() {
        restore();
    }

    void add(CameraBoardSocket socket, const std::vector<uint8_t>& backup) {
        pending[socket] = backup;
    }

    void done(CameraBoardSocket socket) {
        pending.erase(socket);
    }

    // Nothing reached the EEPROMs
    void discard() {
        pending.clear();
    }

    bool has(CameraBoardSocket socket) const {
        return pending.count(socket) != 0;
    }

    // Returns the cameras whose restore failed
    std::vector<CameraBoardSocket> restore() {
        std::vector<CameraBoardSocket> failed;
        for(const auto& [socket, backup] : pending) {
            bool restored = false;
            try {
                Session session({socket});
                session.start();
                session.firstFrames();
                restored = restoreWindow(session.dev(), socket, backup);
            } catch(const std::exception& ex) {
                report("CCM EEPROM " + name(socket) + " restore session failed: " + ex.what());
            }
            if(restored) {
                report("CCM EEPROM " + name(socket) + ": backup in place, checked in a new session");
            } else {
                report("CCM EEPROM RESTORE FAILED: " + name(socket) + ", its first " + std::to_string(backup.size()) + " bytes should have fingerprint "
                       + fingerprint(backup));
                failed.push_back(socket);
            }
        }
        pending.clear();
        return failed;
    }

   private:
    std::map<CameraBoardSocket, std::vector<uint8_t>> pending;
};

// One camera's part of a write round trip
struct RoundTrip {
    CameraBoardSocket socket;
    std::string backupError;  // set when no backup could be taken, in which case nothing was written
    std::vector<uint8_t> backup;
    std::vector<uint8_t> payload;
    Outcome write;
    Outcome readBack;

    // The EEPROM NACKed the first chunk and still holds the backup, as a write-protected EEPROM does
    bool rejected() const {
        return !write.ok && contains(write.error, NOTHING_WRITTEN_ERROR) && readBack.ok && readBack.data == backup;
    }
};

// Backs up the write window of each camera, writes a distinct payload to each, reads the payloads back and restores
// the backups, all in one session. The caller checks the results after PendingRestores::restore(), so a failed check
// can't skip a restore
std::vector<RoundTrip> roundTrip(Session& session, const std::vector<CameraBoardSocket>& sockets, PendingRestores& pending, bool streams) {
    auto& device = session.dev();
    std::vector<RoundTrip> trips;
    bool backedUp = true;
    for(const auto socket : sockets) {
        RoundTrip trip{socket};
        // Two reads that agree, so that a glitch can't become a backup that is later written over valid data
        const auto first = readCcm(device, socket, WRITE_WINDOW);
        const auto second = readCcm(device, socket, WRITE_WINDOW);
        if(!first.ok || !second.ok) {
            trip.backupError = describe(first.ok ? second : first);
        } else if(first.data != second.data) {
            trip.backupError = "two reads differ: " + difference(second.data, first.data);
        } else {
            trip.backup = first.data;
        }
        backedUp = backedUp && trip.backupError.empty();
        trips.push_back(trip);
    }
    if(!backedUp) return trips;

    std::vector<std::vector<uint8_t>> payloads;
    for(auto& trip : trips) {
        trip.payload = makePayload(trip.backup, payloads);
        payloads.push_back(trip.payload);
    }
    bool stalled = false;
    for(auto& trip : trips) {
        stalled = stalled || (streams && !session.streaming());
        if(stalled) {
            trip.write.error = "the stream stalled, so the write wasn't sent";
            continue;
        }
        pending.add(trip.socket, trip.backup);
        trip.write = writeCcm(device, trip.socket, trip.payload);
    }
    for(auto& trip : trips) {
        if(pending.has(trip.socket)) trip.readBack = readCcm(device, trip.socket, WRITE_WINDOW);
    }
    for(auto trip = trips.rbegin(); trip != trips.rend(); ++trip) {
        if(pending.has(trip->socket) && restoreWindow(device, trip->socket, trip->backup)) pending.done(trip->socket);
    }
    for(const auto& trip : trips) {
        report("CCM EEPROM " + name(trip.socket) + ": backup " + (trip.backupError.empty() ? fingerprint(trip.backup) : trip.backupError) + ", write "
               + describe(trip.write) + ", read back "
               + (trip.readBack.ok ? "vs payload: " + difference(trip.readBack.data, trip.payload) : trip.readBack.error) + ", "
               + (pending.has(trip.socket) ? "NOT restored in the session" : "backup in place"));
    }
    return trips;
}

// Write-protected EEPROMs only get a warning. Returns how many EEPROMs accepted the write
size_t checkRoundTrips(const std::vector<RoundTrip>& trips) {
    size_t written = 0;
    for(const auto& trip : trips) {
        INFO(name(trip.socket) << " backup: " << (trip.backupError.empty() ? "OK" : trip.backupError));
        CHECK(trip.backupError.empty());
        if(!trip.backupError.empty()) continue;
        if(trip.rejected()) {
            WARN(name(trip.socket) << " CCM EEPROM rejected the write and is unchanged, so it's write-protected: " << trip.write.error);
            continue;
        }
        ++written;
        INFO(name(trip.socket) << " write: " << describe(trip.write));
        CHECK(trip.write.ok);
        const bool readBackMatches = trip.readBack.ok && trip.readBack.data == trip.payload;
        INFO(name(trip.socket) << " read back: " << (trip.readBack.ok ? difference(trip.readBack.data, trip.payload) : trip.readBack.error));
        CHECK(readBackMatches);
    }
    // Different payloads must read back different: two sockets reaching one EEPROM read back the same one
    for(size_t i = 0; i < trips.size(); ++i) {
        for(size_t j = i + 1; j < trips.size(); ++j) {
            if(!trips[i].readBack.ok || !trips[j].readBack.ok) continue;
            if(trips[i].rejected() || trips[j].rejected()) continue;
            const bool distinct = trips[i].readBack.data != trips[j].readBack.data;
            INFO(name(trips[i].socket) << " and " << name(trips[j].socket) << " read back the same data");
            CHECK(distinct);
        }
    }
    return written;
}

// Cases 1-3: the cameras stream, and each one's CCM EEPROM goes through a write round trip
void writeWhileStreaming(const std::vector<CameraBoardSocket>& sockets, std::chrono::seconds settle) {
    requireCcmEeproms(sockets);
    PendingRestores pending;
    std::vector<RoundTrip> trips;
    bool streamedThrough = false;
    {
        Session session(sockets);
        session.start();
        REQUIRE(session.firstFrames());
        std::this_thread::sleep_for(settle);
        trips = roundTrip(session, sockets, pending, true);
        streamedThrough = session.streaming();
    }
    const auto failedRestores = pending.restore();
    REQUIRE(failedRestores.empty());
    CHECK(streamedThrough);
    if(checkRoundTrips(trips) == 0) SKIP("No CCM EEPROM in this case accepts writes");
}

// Whether CAM_B and CAM_C share an I2C bus, as the firmware's answers in case 4 showed
std::optional<bool> camBCamCShareBus;

// On a shared bus, a CCM EEPROM read with no camera active is refused with a message naming the sockets on the bus
bool firmwareReportsSharedBus(dai::Device& device, CameraBoardSocket socket, CameraBoardSocket other) {
    const auto outcome = readCcm(device, socket, 1);
    return refused(outcome) && contains(outcome.error, name(other));
}

}  // namespace

TEST_CASE("CCM EEPROM readouts are unique", "[ccm]") {
    const auto& inventory = ccmInventory();
    if(inventory.platform != dai::Platform::RVC4) SKIP("CCM EEPROM access is tested on RVC4 only");
    INFO(join(inventory.problems));
    CHECK(inventory.problems.empty());
    if(inventory.eeproms.empty()) WARN("No CCM EEPROM detected on any connected camera");
}

TEST_CASE("1: CCM EEPROM write round trip with CAM_A and CAM_B streaming", "[ccm]") {
    writeWhileStreaming({CAM_A, CAM_B}, 2s);
}

TEST_CASE("2: CCM EEPROM write round trip with CAM_B streaming alone", "[ccm]") {
    writeWhileStreaming({CAM_B}, 2s);
}

TEST_CASE("3: CCM EEPROM write round trip with CAM_C streaming alone", "[ccm]") {
    writeWhileStreaming({CAM_C}, 2s);
}

TEST_CASE("4: CCM EEPROM reads with CAM_B and CAM_C streaming", "[ccm]") {
    requireConnected({CAM_B, CAM_C});
    const auto& inventory = ccmInventory();
    std::map<CameraBoardSocket, std::vector<Outcome>> reads;
    bool streamedThrough = false;
    {
        Session session({CAM_B, CAM_C});
        session.start();
        REQUIRE(session.firstFrames());
        std::this_thread::sleep_for(1s);
        const auto end = Clock::now() + 1s;
        do {
            for(const auto socket : {CAM_B, CAM_C}) reads[socket].push_back(readCcm(session.dev(), socket, WRITE_WINDOW));
        } while(Clock::now() < end);
        streamedThrough = session.streaming();
    }
    // On a shared bus every read is refused. On separate buses every read succeeds, or fails like a module without an EEPROM
    const auto all = [&](CameraBoardSocket socket, bool (*predicate)(const Outcome&)) {
        return std::all_of(reads[socket].begin(), reads[socket].end(), predicate);
    };
    const auto readable = [&](CameraBoardSocket socket) {
        return inventory.eeproms.count(socket) != 0 ? all(socket, [](const Outcome& outcome) { return outcome.ok; }) : all(socket, notPowered);
    };
    const bool shared = all(CAM_B, refused) && all(CAM_C, refused);
    const bool separate = readable(CAM_B) && readable(CAM_C);
    INFO("CAM_B reads: " << summarize(reads[CAM_B]));
    INFO("CAM_C reads: " << summarize(reads[CAM_C]));
    CHECK((shared || separate));
    if(shared || separate) camBCamCShareBus = shared;
    report(std::string(shared     ? "CAM_B and CAM_C share an I2C bus"
                       : separate ? "CAM_B and CAM_C are on separate I2C buses"
                                  : "CAM_B and CAM_C answered inconsistently")
           + ". CAM_B reads: " + summarize(reads[CAM_B]) + ". CAM_C reads: " + summarize(reads[CAM_C]));
    CHECK(streamedThrough);
}

TEST_CASE("5: CCM EEPROM writes with CAM_B and CAM_C streaming", "[ccm]") {
    requireCcmEeproms({CAM_B, CAM_C});
    PendingRestores pending;
    Outcome probeB;
    Outcome probeC;
    std::vector<Outcome> writes;
    bool streamedThrough = false;
    {
        Session session({CAM_B, CAM_C});
        session.start();
        REQUIRE(session.firstFrames());
        std::this_thread::sleep_for(1s);
        // Writes go out only after reads in this session were refused: the same firmware check guards both
        probeB = readCcm(session.dev(), CAM_B, WRITE_WINDOW);
        probeC = readCcm(session.dev(), CAM_C, WRITE_WINDOW);
        if(refused(probeB) && refused(probeC)) {
            // The backups come from the inventory, read while each camera streamed alone
            std::map<CameraBoardSocket, std::vector<uint8_t>> payloads;
            std::vector<std::vector<uint8_t>> taken;
            for(const auto socket : {CAM_B, CAM_C}) {
                const auto backup = inventoryWindow(socket);
                payloads[socket] = makePayload(backup, taken);
                taken.push_back(payloads[socket]);
                pending.add(socket, backup);
            }
            const auto end = Clock::now() + 1s;
            do {
                for(const auto socket : {CAM_B, CAM_C}) writes.push_back(writeCcm(session.dev(), socket, payloads[socket]));
            } while(Clock::now() < end);
            streamedThrough = session.streaming();
        }
    }
    if(!refused(probeB) || !refused(probeC)) {
        INFO("CAM_B read: " << describe(probeB) << "; CAM_C read: " << describe(probeC));
        REQUIRE((!refused(probeB) && !refused(probeC)));
        SKIP("CAM_B and CAM_C are on separate I2C buses, so no write is refused. Case 1 covers writes on separate buses");
    }
    // A refused write sends nothing to the EEPROM, so only writes that went through need restoring
    const bool allRefused = std::all_of(writes.begin(), writes.end(), refused);
    if(allRefused) pending.discard();
    const auto failedRestores = pending.restore();
    REQUIRE(failedRestores.empty());
    report("CAM_B and CAM_C writes: " + summarize(writes));
    INFO("Writes: " << summarize(writes));
    CHECK(allRefused);
    CHECK(streamedThrough);
}

TEST_CASE("6: CCM EEPROM read right after the stream starts", "[ccm]") {
    requireConnected({CAM_A});
    const auto& inventory = ccmInventory();
    Outcome early;
    bool cameUp = false;
    bool streamedThrough = false;
    {
        Session session({CAM_A});
        session.start();
        // No wait: the read races the sensor's power-up and probe
        early = readCcm(session.dev(), CAM_A, WRITE_WINDOW);
        cameUp = session.firstFrames();
        streamedThrough = cameUp && session.streaming();
    }
    report("CAM_A read right after the start: " + describe(early) + (cameUp ? ", the stream came up" : ", the stream did NOT come up"));
    INFO("CAM_A read: " << describe(early));
    CHECK((early.ok || notPowered(early)));
    if(early.ok && inventory.eeproms.count(CAM_A) != 0) {
        const bool matchesInventory = early.data == inventoryWindow(CAM_A);
        CHECK(matchesInventory);
    }
    REQUIRE(cameUp);
    CHECK(streamedThrough);
}

TEST_CASE("7: CCM EEPROM access before the pipeline starts", "[ccm]") {
    requireConnected({CAM_A});
    const auto& inventory = ccmInventory();
    const bool hasCamB = std::find(inventory.connected.begin(), inventory.connected.end(), CAM_B) != inventory.connected.end();
    PendingRestores pending;
    Outcome readA;
    Outcome readB;
    Outcome writeA;
    std::vector<RoundTrip> trips;
    {
        // The pipeline is built but not started, so the device has created no camera: none is active
        Session session({CAM_A});
        readA = readCcm(session.dev(), CAM_A, WRITE_WINDOW);
        if(hasCamB) readB = readCcm(session.dev(), CAM_B, WRITE_WINDOW);
        if(readA.ok) {
            // The module is powered without streaming (as on OAK4-FFC), so writing works too
            trips = roundTrip(session, {CAM_A}, pending, false);
        } else if(inventory.eeproms.count(CAM_A) != 0) {
            // The write must fail like the read. The backup is the inventory readout, in case it doesn't
            const auto backup = inventoryWindow(CAM_A);
            pending.add(CAM_A, backup);
            writeA = writeCcm(session.dev(), CAM_A, makePayload(backup, {}));
        }
    }
    const auto failedRestores = pending.restore();
    REQUIRE(failedRestores.empty());
    report("Before the start: CAM_A read " + describe(readA) + (hasCamB ? ", CAM_B read " + describe(readB) : ""));
    INFO("CAM_A read: " << describe(readA));
    CHECK((readA.ok || notPowered(readA)));
    if(readA.ok) {
        checkRoundTrips(trips);
    } else if(inventory.eeproms.count(CAM_A) != 0) {
        INFO("CAM_A write: " << describe(writeA));
        CHECK(notPowered(writeA));
    }
    if(hasCamB) {
        INFO("CAM_B read: " << describe(readB));
        if(!camBCamCShareBus.has_value()) {
            CHECK((refused(readB) || readB.ok || notPowered(readB)));
        } else if(*camBCamCShareBus) {
            CHECK((refused(readB) && contains(readB.error, "(active: none)")));
        } else {
            CHECK((readB.ok || notPowered(readB)));
        }
    }
}

// The CBA EEPROM holds the camera board's calibration, so the test only reads it: the API can't put it back exactly
// after a write. A read returns the calibration with IMU extrinsics patched in, flashing drops the miscellaneous data,
// and there is no raw CBA read to verify a restore with
TEST_CASE("CBA EEPROM reads with no camera streaming", "[cba]") {
    // No pipeline: no sensor is active or using a camera I2C bus
    dai::Device device;
    if(device.getPlatform() != dai::Platform::RVC4) SKIP("CBA EEPROM access is tested on RVC4 only");
    std::map<CameraBoardSocket, std::string> calibrations;
    std::vector<CameraBoardSocket> detected;
    std::vector<std::string> problems;
    for(const auto& features : device.getConnectedCameraFeatures()) {
        const auto socket = features.socket;
        try {
            if(!device.isCBAEepromAvailable(socket)) {
                report("CBA EEPROM " + name(socket) + ": not detected");
                continue;
            }
            detected.push_back(socket);
            const auto first = device.readCBACalibration2(socket).eepromToJson().dump();
            const auto second = device.readCBACalibration2(socket).eepromToJson().dump();
            if(first != second) {
                problems.push_back(name(socket) + " read differently twice");
                continue;
            }
            report("CBA EEPROM " + name(socket) + ": calibration read, fingerprint " + fingerprint(first));
            calibrations[socket] = first;
        } catch(const dai::EepromError& ex) {
            report("CBA EEPROM " + name(socket) + ": detected, no calibration read: " + ex.what());
        } catch(const std::exception& ex) {
            problems.push_back(name(socket) + ": " + ex.what());
        }
    }
    // Sockets on one I2C bus share its CBA EEPROM. Otherwise identical readouts mean two sockets reach the same EEPROM
    for(auto a = calibrations.begin(); a != calibrations.end(); ++a) {
        for(auto b = std::next(a); b != calibrations.end(); ++b) {
            if(a->second == b->second && !firmwareReportsSharedBus(device, a->first, b->first)) {
                problems.push_back(name(a->first) + " and " + name(b->first) + " read identical calibrations but aren't on a shared I2C bus");
            }
        }
    }
    for(const auto& problem : problems) report("CBA EEPROM problem: " + problem);
    INFO(join(problems));
    CHECK(problems.empty());
    if(detected.empty()) SKIP("No CBA EEPROM detected");
}
