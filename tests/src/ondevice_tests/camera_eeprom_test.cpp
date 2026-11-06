// RVC4 camera EEPROM access:
// - the camera module (CCM) EEPROM through readCcmEepromRaw() / writeCcmEepromRaw(). It is accessed while its camera
//   streams, since the module may be unpowered otherwise. On an I2C bus shared by several camera sockets the firmware
//   allows access only while the requested camera is the only active one on the bus. The test learns which sockets
//   share a bus from the firmware's refusals, so it covers shared buses on any sockets and with any number of them.
//   A camera without a CCM EEPROM answers like an unpowered module, so cases that need an answer from the EEPROM skip
//   such cameras. The shared-bus refusal cases run without EEPROMs, since the firmware refuses before any I2C traffic
// A camera may have a CCM EEPROM, a CBA EEPROM or both. The cases for one type skip when no camera has it, and a
// separate case fails for a camera with neither
// - the camera board (CBA) EEPROM through the CBA calibration API, read only and with no camera streaming (see its test
//   case)
// The test controls which cameras stream, and never more than two at once. Every write is restored, and EEPROM contents
// never reach the log: only fingerprints, offsets and counts do.
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
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "depthai/depthai.hpp"
#include "depthai/device/EepromError.hpp"

using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;
using dai::CameraBoardSocket;

namespace {

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

std::set<std::string> names(const std::vector<CameraBoardSocket>& sockets) {
    std::set<std::string> result;
    for(const auto socket : sockets) result.insert(name(socket));
    return result;
}

std::string join(const std::set<std::string>& names) {
    std::string text;
    for(const auto& item : names) text += (text.empty() ? "" : ", ") + item;
    return text;
}

// The firmware lists sockets as "CAM_B, CAM_C", and a port without a socket as "CSI<n>"
std::set<std::string> splitNames(const std::string& text) {
    std::set<std::string> result;
    size_t start = 0;
    while(true) {
        const auto end = text.find(", ", start);
        result.insert(text.substr(start, end == std::string::npos ? std::string::npos : end - start));
        if(end == std::string::npos) return result;
        start = end + 2;
    }
}

// The text between two markers
std::optional<std::string> between(const std::string& text, const std::string& before, const std::string& after) {
    const auto start = text.find(before);
    if(start == std::string::npos) return std::nullopt;
    const auto from = start + before.size();
    const auto end = text.find(after, from);
    if(end == std::string::npos) return std::nullopt;
    return text.substr(from, end - from);
}

// A shared-bus refusal, worded by the firmware as "CCM EEPROM of <socket> is not accessible: I2C bus <n> is shared by
// <sockets>, so it must be the only active camera on it (active: <sockets> | none)"
struct Refusal {
    int bus = -1;
    std::set<std::string> sockets;
    std::set<std::string> active;
};

std::optional<Refusal> parseRefusal(const Outcome& outcome) {
    if(!refused(outcome)) return std::nullopt;
    const auto bus = between(outcome.error, "I2C bus ", " is shared by ");
    const auto sockets = between(outcome.error, " is shared by ", ", so it ");
    const auto active = between(outcome.error, "(active: ", ")");
    if(!bus || !sockets || !active) return std::nullopt;
    Refusal refusal;
    try {
        refusal.bus = std::stoi(*bus);
    } catch(const std::exception&) {
        return std::nullopt;
    }
    refusal.sockets = splitNames(*sockets);
    if(*active != "none") refusal.active = splitNames(*active);
    return refusal;
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

// The I2C buses of the connected cameras, as the firmware reports them. With no camera active, a CCM EEPROM access on a
// shared bus is refused with an error naming the bus and every socket on it, while a camera alone on its bus isn't
// refused. Learned once, with no pipeline, so it doesn't depend on which cases ran before. The CBA EEPROMs are
// probed with no camera active too, so the same connection notes which cameras have one
struct BusLayout {
    dai::Platform platform{};
    std::vector<CameraBoardSocket> connected;
    std::map<int, std::set<std::string>> named;            // shared bus -> every socket the firmware names on it
    std::map<int, std::vector<CameraBoardSocket>> shared;  // shared bus -> its connected cameras
    std::map<CameraBoardSocket, int> busOf;                // connected camera on a shared bus -> that bus
    std::set<CameraBoardSocket> cbaEeproms;                // connected cameras with a CBA EEPROM
    std::vector<std::string> problems;

    bool sameBus(CameraBoardSocket a, CameraBoardSocket b) const {
        const auto busA = busOf.find(a);
        const auto busB = busOf.find(b);
        return busA != busOf.end() && busB != busOf.end() && busA->second == busB->second;
    }
};

// Adds one camera's answer to a read with no camera active
void addToLayout(BusLayout& layout, CameraBoardSocket socket, const Outcome& outcome) {
    if(!refused(outcome)) {
        // Alone on its bus, so the read goes out. Without streaming the module may be unpowered
        if(!outcome.ok && !notPowered(outcome)) layout.problems.push_back(name(socket) + " read with no camera active failed: " + outcome.error);
        return;
    }
    const auto refusal = parseRefusal(outcome);
    if(!refusal) {
        layout.problems.push_back(name(socket) + " was refused without naming its bus, the sockets on it and the active cameras: " + outcome.error);
        return;
    }
    if(!refusal->active.empty()) layout.problems.push_back(name(socket) + " was refused for active cameras although none is active: " + outcome.error);
    if(refusal->sockets.count(name(socket)) == 0) layout.problems.push_back(name(socket) + " was refused for a bus it isn't named on: " + outcome.error);
    const auto named = layout.named.emplace(refusal->bus, refusal->sockets);
    if(!named.second && named.first->second != refusal->sockets) {
        layout.problems.push_back("I2C bus " + std::to_string(refusal->bus) + " is named with different sockets: " + join(named.first->second) + " and "
                                  + join(refusal->sockets));
    }
    layout.busOf[socket] = refusal->bus;
    layout.shared[refusal->bus].push_back(socket);
}

// The given cameras with a CBA EEPROM, as the firmware detected it at startup
std::set<CameraBoardSocket> withCbaEeprom(dai::Device& device, const std::vector<CameraBoardSocket>& sockets) {
    std::set<CameraBoardSocket> result;
    for(const auto socket : sockets) {
        if(device.isCBAEepromAvailable(socket)) result.insert(socket);
    }
    return result;
}

const BusLayout& busLayout() {
    static const BusLayout layout = [] {
        BusLayout result;
        // No pipeline, so the device has created no camera: none is active
        dai::Device device;
        result.platform = device.getPlatform();
        if(result.platform != dai::Platform::RVC4) return result;
        for(const auto& features : device.getConnectedCameraFeatures()) result.connected.push_back(features.socket);
        for(const auto socket : result.connected) addToLayout(result, socket, readCcm(device, socket, 1));
        result.cbaEeproms = withCbaEeprom(device, result.connected);
        for(const auto& [bus, sockets] : result.named) {
            for(const auto socket : result.connected) {
                if(sockets.count(name(socket)) != 0 && result.busOf.count(socket) == 0) {
                    result.problems.push_back(name(socket) + " is named on shared I2C bus " + std::to_string(bus) + " but wasn't refused");
                }
            }
            report("CCM EEPROM I2C bus " + std::to_string(bus) + ": shared by " + join(sockets) + ", connected " + join(names(result.shared[bus])));
        }
        for(const auto socket : result.connected) {
            if(result.busOf.count(socket) == 0) report("CCM EEPROM " + name(socket) + ": alone on its I2C bus");
        }
        for(const auto& problem : result.problems) report("CCM EEPROM bus layout problem: " + problem);
        return result;
    }();
    return layout;
}

// An access on a shared bus must be refused, naming that bus, its sockets and exactly the active cameras
bool refusedWith(const BusLayout& layout, const Outcome& outcome, int bus, const std::vector<CameraBoardSocket>& active) {
    const auto refusal = parseRefusal(outcome);
    return refusal && refusal->bus == bus && refusal->sockets == layout.named.at(bus) && refusal->active == names(active);
}

// Two connected cameras on one shared bus
struct SharedPair {
    int bus;
    CameraBoardSocket a;
    CameraBoardSocket b;
};

std::vector<SharedPair> sharedPairs(const BusLayout& layout) {
    std::vector<SharedPair> pairs;
    for(const auto& [bus, sockets] : layout.shared) {
        for(size_t i = 0; i < sockets.size(); ++i) {
            for(size_t j = i + 1; j < sockets.size(); ++j) pairs.push_back({bus, sockets[i], sockets[j]});
        }
    }
    return pairs;
}

// Cases that pick cameras by their bus need a layout without problems
const BusLayout& requireLayout() {
    const auto& layout = busLayout();
    if(layout.platform != dai::Platform::RVC4) SKIP("CCM EEPROM access is tested on RVC4 only");
    INFO(join(layout.problems));
    REQUIRE(layout.problems.empty());
    return layout;
}

// Skips when no two connected cameras share a bus
const BusLayout& requireSharedBus() {
    const auto& layout = requireLayout();
    if(sharedPairs(layout).empty()) SKIP("No two connected cameras share an I2C bus");
    return layout;
}

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

// The given cameras that have a CCM EEPROM. Without one a camera answers like an unpowered module, so a case couldn't
// tell a missing EEPROM from broken access: the others are logged and not tested
std::vector<CameraBoardSocket> withCcmEeprom(const CcmInventory& inventory, const std::vector<CameraBoardSocket>& sockets) {
    std::vector<CameraBoardSocket> result;
    for(const auto socket : sockets) {
        if(inventory.eeproms.count(socket) != 0) {
            result.push_back(socket);
        } else {
            report(name(socket) + ": no CCM EEPROM, not tested");
        }
    }
    return result;
}

std::string ccmEepromList(const CcmInventory& inventory) {
    std::set<std::string> sockets;
    for(const auto& entry : inventory.eeproms) sockets.insert(name(entry.first));
    return sockets.empty() ? "none" : join(sockets);
}

// Writing needs readouts the inventory could trust: otherwise a restore could put one EEPROM's contents into another
const CcmInventory& requireTrustedInventory() {
    const auto& inventory = ccmInventory();
    if(inventory.platform != dai::Platform::RVC4) SKIP("CCM EEPROM access is tested on RVC4 only");
    INFO(join(inventory.problems));
    REQUIRE(inventory.problems.empty());
    return inventory;
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

// The cameras stream, and each one's CCM EEPROM goes through a write round trip. Returns how many EEPROMs accepted the write
size_t writeWhileStreaming(const std::vector<CameraBoardSocket>& sockets, std::chrono::seconds settle) {
    PendingRestores pending;
    std::vector<RoundTrip> trips;
    bool cameUp = false;
    bool streamedThrough = false;
    {
        Session session(sockets);
        session.start();
        cameUp = session.firstFrames();
        if(cameUp) {
            std::this_thread::sleep_for(settle);
            trips = roundTrip(session, sockets, pending, true);
            streamedThrough = session.streaming();
        }
    }
    const auto failedRestores = pending.restore();
    REQUIRE(failedRestores.empty());
    REQUIRE(cameUp);
    CHECK(streamedThrough);
    return checkRoundTrips(trips);
}

// Two cameras with a CCM EEPROM on different I2C buses
std::optional<std::pair<CameraBoardSocket, CameraBoardSocket>> pairOnDifferentBuses(const BusLayout& layout, const CcmInventory& inventory) {
    for(auto a = inventory.eeproms.begin(); a != inventory.eeproms.end(); ++a) {
        for(auto b = std::next(a); b != inventory.eeproms.end(); ++b) {
            if(!layout.sameBus(a->first, b->first)) return std::make_pair(a->first, b->first);
        }
    }
    return std::nullopt;
}

}  // namespace

TEST_CASE("CCM EEPROM I2C bus layout", "[ccm]") {
    // Before any camera is active, every camera on a shared bus is refused with "(active: none)"
    const auto& layout = busLayout();
    if(layout.platform != dai::Platform::RVC4) SKIP("CCM EEPROM access is tested on RVC4 only");
    INFO(join(layout.problems));
    CHECK(layout.problems.empty());
    if(layout.shared.empty()) report("No connected camera shares its I2C bus");
}

TEST_CASE("CCM EEPROM readouts are unique", "[ccm]") {
    const auto& inventory = ccmInventory();
    if(inventory.platform != dai::Platform::RVC4) SKIP("CCM EEPROM access is tested on RVC4 only");
    INFO(join(inventory.problems));
    CHECK(inventory.problems.empty());
    if(inventory.eeproms.empty()) SKIP("No CCM EEPROM detected on any connected camera, so there are no readouts to compare");
}

TEST_CASE("Every connected camera has a CCM or CBA EEPROM", "[ccm][cba]") {
    const auto& layout = requireLayout();
    const auto& inventory = ccmInventory();
    std::vector<std::string> without;
    for(const auto socket : layout.connected) {
        const bool ccm = inventory.eeproms.count(socket) != 0;
        const bool cba = layout.cbaEeproms.count(socket) != 0;
        report("EEPROMs of " + name(socket) + ": " + (ccm && cba ? "CCM and CBA" : ccm ? "CCM" : cba ? "CBA" : "none"));
        if(!ccm && !cba) without.push_back(name(socket));
    }
    INFO("Cameras without a CCM or CBA EEPROM: " << join(without));
    CHECK(without.empty());
}

TEST_CASE("1: CCM EEPROM write round trip with two cameras on different I2C buses streaming", "[ccm]") {
    // Each camera is the only active one on its bus, so neither is refused
    const auto& layout = requireLayout();
    const auto& inventory = requireTrustedInventory();
    const auto pair = pairOnDifferentBuses(layout, inventory);
    if(!pair) SKIP("No two cameras with a CCM EEPROM are on different I2C buses (CCM EEPROMs: " << ccmEepromList(inventory) << ")");
    if(writeWhileStreaming({pair->first, pair->second}, 2s) == 0) SKIP("No CCM EEPROM in this case accepts writes");
}

TEST_CASE("2: CCM EEPROM write round trip with each camera streaming alone", "[ccm]") {
    // On a shared bus too: the camera is then the only active one on it
    const auto& inventory = requireTrustedInventory();
    const auto tested = withCcmEeprom(inventory, inventory.connected);
    if(tested.empty()) SKIP("No CCM EEPROM detected on any connected camera");
    size_t written = 0;
    for(const auto socket : tested) {
        INFO(name(socket) << " streams alone");
        written += writeWhileStreaming({socket}, 2s);
    }
    if(written == 0) SKIP("No CCM EEPROM accepts writes");
}

TEST_CASE("3: CCM EEPROM reads of idle cameras on a shared I2C bus while another camera on it streams", "[ccm]") {
    // With one camera powered, 0x50 on the bus is that camera's EEPROM, so reading another socket's must be refused
    const auto& layout = requireSharedBus();
    for(const auto& [bus, sockets] : layout.shared) {
        if(sockets.size() < 2) continue;
        for(const auto active : sockets) {
            std::map<CameraBoardSocket, Outcome> reads;
            bool cameUp = false;
            bool streamedThrough = false;
            {
                Session session({active});
                session.start();
                cameUp = session.firstFrames();
                if(cameUp) {
                    std::this_thread::sleep_for(1s);
                    for(const auto idle : sockets) {
                        if(idle != active) reads[idle] = readCcm(session.dev(), idle, WRITE_WINDOW);
                    }
                    streamedThrough = session.streaming();
                }
            }
            INFO("I2C bus " << bus << ": " << name(active) << " streams alone");
            REQUIRE(cameUp);
            for(const auto& [idle, read] : reads) {
                report("I2C bus " + std::to_string(bus) + ", " + name(active) + " streaming: " + name(idle) + " read " + describe(read));
                INFO(name(idle) << " read: " << describe(read));
                CHECK(refusedWith(layout, read, bus, {active}));
            }
            CHECK(streamedThrough);
        }
    }
}

TEST_CASE("4: CCM EEPROM reads with two cameras on a shared I2C bus streaming", "[ccm]") {
    const auto& layout = requireSharedBus();
    for(const auto& pair : sharedPairs(layout)) {
        const std::vector<CameraBoardSocket> sockets{pair.a, pair.b};
        std::map<CameraBoardSocket, std::vector<Outcome>> reads;
        bool cameUp = false;
        bool streamedThrough = false;
        {
            Session session(sockets);
            session.start();
            cameUp = session.firstFrames();
            if(cameUp) {
                std::this_thread::sleep_for(1s);
                const auto end = Clock::now() + 1s;
                do {
                    for(const auto socket : sockets) reads[socket].push_back(readCcm(session.dev(), socket, WRITE_WINDOW));
                } while(Clock::now() < end);
                streamedThrough = session.streaming();
            }
        }
        INFO("I2C bus " << pair.bus << ": " << name(pair.a) << " and " << name(pair.b) << " stream");
        REQUIRE(cameUp);
        for(const auto socket : sockets) {
            const auto& outcomes = reads[socket];
            const bool allRefused =
                std::all_of(outcomes.begin(), outcomes.end(), [&](const Outcome& outcome) { return refusedWith(layout, outcome, pair.bus, sockets); });
            report("I2C bus " + std::to_string(pair.bus) + ", " + name(pair.a) + " and " + name(pair.b) + " streaming: " + name(socket)
                   + " reads: " + summarize(outcomes));
            INFO(name(socket) << " reads: " << summarize(outcomes));
            CHECK(allRefused);
        }
        CHECK(streamedThrough);
    }
}

TEST_CASE("5: CCM EEPROM writes with two cameras on a shared I2C bus streaming", "[ccm]") {
    // The refusal comes before any I2C traffic, so the cameras need no EEPROM
    const auto& layout = requireSharedBus();
    const auto& inventory = requireTrustedInventory();
    for(const auto& pair : sharedPairs(layout)) {
        const std::vector<CameraBoardSocket> sockets{pair.a, pair.b};
        PendingRestores pending;
        std::map<CameraBoardSocket, Outcome> probes;
        std::vector<Outcome> writes;
        bool cameUp = false;
        bool streamedThrough = false;
        {
            Session session(sockets);
            session.start();
            cameUp = session.firstFrames();
            if(cameUp) {
                std::this_thread::sleep_for(1s);
                // Writes go out only after reads in this session were refused: the same firmware check guards both
                for(const auto socket : sockets) probes[socket] = readCcm(session.dev(), socket, WRITE_WINDOW);
                if(refusedWith(layout, probes[pair.a], pair.bus, sockets) && refusedWith(layout, probes[pair.b], pair.bus, sockets)) {
                    // A write that went through would reach the EEPROMs of both powered modules, so both get backups. They
                    // come from the inventory, read while each camera streamed alone. A module without an EEPROM has none
                    std::map<CameraBoardSocket, std::vector<uint8_t>> payloads;
                    std::vector<std::vector<uint8_t>> taken;
                    for(const auto socket : sockets) {
                        const bool hasEeprom = inventory.eeproms.count(socket) != 0;
                        const auto backup = hasEeprom ? inventoryWindow(socket) : std::vector<uint8_t>(WRITE_WINDOW, 0xFF);
                        payloads[socket] = makePayload(backup, taken);
                        taken.push_back(payloads[socket]);
                        if(hasEeprom) pending.add(socket, backup);
                    }
                    const auto end = Clock::now() + 1s;
                    do {
                        for(const auto socket : sockets) writes.push_back(writeCcm(session.dev(), socket, payloads[socket]));
                    } while(Clock::now() < end);
                }
                streamedThrough = session.streaming();
            }
        }
        // A refused write sends nothing to the EEPROM, so only writes that went through need restoring
        const bool allRefused =
            !writes.empty() && std::all_of(writes.begin(), writes.end(), [&](const Outcome& write) { return refusedWith(layout, write, pair.bus, sockets); });
        if(allRefused) pending.discard();
        const auto failedRestores = pending.restore();
        REQUIRE(failedRestores.empty());
        INFO("I2C bus " << pair.bus << ": " << name(pair.a) << " and " << name(pair.b) << " stream");
        REQUIRE(cameUp);
        for(const auto socket : sockets) {
            INFO(name(socket) << " read: " << describe(probes[socket]));
            CHECK(refusedWith(layout, probes[socket], pair.bus, sockets));
        }
        report("I2C bus " + std::to_string(pair.bus) + ", " + name(pair.a) + " and " + name(pair.b) + " streaming: writes " + summarize(writes));
        INFO("Writes: " << summarize(writes));
        CHECK(allRefused);
        CHECK(streamedThrough);
    }
}

TEST_CASE("6: CCM EEPROM read right after the stream starts", "[ccm]") {
    const auto& inventory = ccmInventory();
    if(inventory.platform != dai::Platform::RVC4) SKIP("CCM EEPROM access is tested on RVC4 only");
    if(inventory.eeproms.empty()) SKIP("No CCM EEPROM detected on any connected camera");
    // The first camera with a CCM EEPROM
    const auto socket = inventory.eeproms.begin()->first;
    Outcome early;
    bool cameUp = false;
    bool streamedThrough = false;
    {
        Session session({socket});
        session.start();
        // No wait: the read races the sensor's power-up and probe
        early = readCcm(session.dev(), socket, WRITE_WINDOW);
        cameUp = session.firstFrames();
        streamedThrough = cameUp && session.streaming();
    }
    report(name(socket) + " read right after the start: " + describe(early) + (cameUp ? ", the stream came up" : ", the stream did NOT come up"));
    INFO(name(socket) << " read: " << describe(early));
    // The module may still be powering up
    CHECK((early.ok || notPowered(early)));
    if(early.ok) {
        const bool matchesInventory = early.data == inventoryWindow(socket);
        CHECK(matchesInventory);
    }
    REQUIRE(cameUp);
    CHECK(streamedThrough);
}

TEST_CASE("7: CCM EEPROM access before the pipeline starts", "[ccm]") {
    // Cameras on a shared bus are refused before the start: the bus layout case checks that
    const auto& layout = requireLayout();
    const auto& inventory = ccmInventory();
    std::vector<CameraBoardSocket> alone;
    for(const auto socket : layout.connected) {
        if(layout.busOf.count(socket) == 0) alone.push_back(socket);
    }
    if(alone.empty()) SKIP("Every connected camera shares its I2C bus, so the bus layout case covers access before the start");
    const auto tested = withCcmEeprom(inventory, alone);
    if(tested.empty()) SKIP("No camera alone on its I2C bus has a CCM EEPROM (alone: " << join(names(alone)) << ")");
    PendingRestores pending;
    std::map<CameraBoardSocket, Outcome> reads;
    std::map<CameraBoardSocket, Outcome> writes;
    std::vector<RoundTrip> trips;
    {
        // The pipeline is built but not started, so the device has created no camera: none is active
        Session session(tested);
        std::vector<CameraBoardSocket> readable;
        for(const auto socket : tested) {
            reads[socket] = readCcm(session.dev(), socket, WRITE_WINDOW);
            if(reads[socket].ok) {
                readable.push_back(socket);
            } else {
                // The write must fail like the read. The backup is the inventory readout, in case it doesn't
                const auto backup = inventoryWindow(socket);
                pending.add(socket, backup);
                writes[socket] = writeCcm(session.dev(), socket, makePayload(backup, {}));
            }
        }
        // A module powered without streaming (as on OAK4-FFC) can be written too
        trips = roundTrip(session, readable, pending, false);
    }
    const auto failedRestores = pending.restore();
    REQUIRE(failedRestores.empty());
    for(const auto& [socket, read] : reads) {
        report("Before the start: " + name(socket) + " read " + describe(read));
        INFO(name(socket) << " read: " << describe(read));
        CHECK((read.ok || notPowered(read)));
    }
    for(const auto& [socket, write] : writes) {
        INFO(name(socket) << " write: " << describe(write));
        CHECK(notPowered(write));
    }
    checkRoundTrips(trips);
}

// The CBA EEPROM holds the camera board's calibration, so the test only reads it: the API can't put it back exactly
// after a write. A read returns the calibration with IMU extrinsics patched in, flashing drops the miscellaneous data,
// and there is no raw CBA read to verify a restore with
TEST_CASE("CBA EEPROM reads with no camera streaming", "[cba]") {
    // Before the device below connects: the layout uses its own connection
    const auto& layout = busLayout();
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
            if(a->second == b->second && !layout.sameBus(a->first, b->first)) {
                problems.push_back(name(a->first) + " and " + name(b->first) + " read identical calibrations but aren't on a shared I2C bus");
            }
        }
    }
    for(const auto& problem : problems) report("CBA EEPROM problem: " + problem);
    INFO(join(problems));
    CHECK(problems.empty());
    if(detected.empty()) SKIP("No CBA EEPROM detected");
}
