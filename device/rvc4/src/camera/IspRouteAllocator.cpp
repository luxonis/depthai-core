#include "IspRouteAllocator.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <optional>

#include <spdlog/spdlog.h>

namespace dai {

namespace {

// Pixel-rate budgets. A shared offline pool serves its members sequentially,
// so its usable rate is the engine's peak (785 MHz, ~1 pix/clk) derated for
// the serving overhead between frames. BPS strips wide frames internally but
// tops out lower once striping overlap is counted.
constexpr double kPoolBudgetPixPerSec = 550e6;
constexpr double kBpsBudgetPixPerSec = 650e6;
// Widest frame a single IFE processes inline; above this a sensor needs a
// dual-IFE pair or BPS.
constexpr uint32_t kMaxSingleIfeWidth = 5760;

constexpr int kNumFullIfe = 3;
// Only IFE0/IFE1 can host shared pools: the offline fetch is SFE BUS_RD ->
// same-index CSID -> IFE, and there is no SFE2.
constexpr int kNumPoolIfe = 2;

// Ingress slots for pool/BPS members, in assignment preference order. The
// lites are otherwise idle; SFE RDI writes coexist with the SFE's BUS_RD
// fetch duty, but SFE_n shares CSID_n's receiver with a dedicated pixel path
// on IFE_n, so a dedicated IFE_n blocks sfe_n ingress.
const std::array<const char*, 4> kIngressNames = {"ife-lite0", "ife-lite1", "sfe0", "sfe1"};
constexpr int kIngressSfe0 = 2;
constexpr int kIngressSfe1 = 3;

std::string normalizeToken(const std::string& token) {
    std::string normalized;
    for(const char c : token) {
        if(c == '-' || c == '_' || c == ' ') {
            continue;
        }
        normalized += static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }
    return normalized;
}

std::vector<std::string> splitTokens(const std::string& value) {
    std::vector<std::string> tokens;
    std::string current;
    for(const char c : value) {
        if(c == '+' || c == ',') {
            if(!current.empty()) {
                tokens.push_back(current);
            }
            current.clear();
        } else {
            current += c;
        }
    }
    if(!current.empty()) {
        tokens.push_back(current);
    }
    return tokens;
}

double sensorLoad(const IspSensorRequest& req) {
    return static_cast<double>(req.width) * static_cast<double>(req.height) * static_cast<double>(req.fps);
}

struct ParsedShared {
    int ingress{-1};       // index into kIngressNames, -1 = unknown token
    std::vector<int> ifes;  // processor IFE indices
    bool bps{false};
    bool valid{false};
};

ParsedShared parseSharedIsp(const std::string& value) {
    ParsedShared parsed;
    const auto tokens = splitTokens(value);
    if(tokens.size() < 2) {
        return parsed;
    }
    const std::string ingress = normalizeToken(tokens[0]);
    for(size_t i = 0; i < kIngressNames.size(); i++) {
        if(normalizeToken(kIngressNames[i]) == ingress) {
            parsed.ingress = static_cast<int>(i);
        }
    }
    for(size_t i = 1; i < tokens.size(); i++) {
        const std::string processor = normalizeToken(tokens[i]);
        if(processor == "bps" || processor == "bsp") {
            parsed.bps = true;
        } else if(processor.size() == 4 && processor.rfind("ife", 0) == 0 && processor[3] >= '0' && processor[3] <= '2') {
            parsed.ifes.push_back(processor[3] - '0');
        } else {
            return parsed;
        }
    }
    parsed.valid = !tokens.empty() && (parsed.bps || !parsed.ifes.empty());
    return parsed;
}

std::string canonicalShared(const ParsedShared& parsed) {
    std::string canonical = parsed.ingress >= 0 ? kIngressNames[parsed.ingress] : "?";
    for(const int ife : parsed.ifes) {
        canonical += "+ife" + std::to_string(ife);
    }
    if(parsed.bps) {
        canonical += "+bps";
    }
    return canonical;
}

std::string canonicalFullIfe(const std::vector<int>& ifes) {
    std::string value;
    for(const int ife : ifes) {
        if(!value.empty()) {
            value += ",";
        }
        value += std::to_string(ife);
    }
    return "full-ife:" + value;
}

struct FleetState {
    bool ifeDedicated[kNumFullIfe]{};
    bool ifePool[kNumPoolIfe]{};
    double poolLoad[kNumPoolIfe]{};
    bool ingressUsed[4]{};
    double bpsLoad{0.0};

    bool ifeFree(int n) const {
        if(n < kNumPoolIfe && ifePool[n]) {
            return false;
        }
        return !ifeDedicated[n];
    }
    bool ingressAvailable(int slot) const {
        if(ingressUsed[slot]) {
            return false;
        }
        if(slot == kIngressSfe0 && ifeDedicated[0]) {
            return false;
        }
        if(slot == kIngressSfe1 && ifeDedicated[1]) {
            return false;
        }
        return true;
    }
};

IspRouteDecision manualDecision(const IspSensorRequest& req) {
    IspRouteDecision decision;
    decision.automatic = false;
    if(!req.manualFullIfe.empty()) {
        decision.fullIfe = req.manualFullIfe;
        std::vector<int> ifes;
        for(const auto& token : splitTokens(req.manualFullIfe)) {
            const std::string normalized = normalizeToken(token);
            if(normalized.size() == 1 && normalized[0] >= '0' && normalized[0] <= '2') {
                ifes.push_back(normalized[0] - '0');
            }
        }
        decision.canonical = ifes.empty() ? "full-ife:?" : canonicalFullIfe(ifes);
    } else {
        decision.sharedIsp = req.manualSharedIsp;
        const auto parsed = parseSharedIsp(req.manualSharedIsp);
        decision.canonical = parsed.valid ? canonicalShared(parsed) : "invalid";
    }
    return decision;
}

// Result of trying to pack a set of automatic sensors into the open pools
// (and, only when pools cannot take them, BPS).
struct Packing {
    bool ok{false};
    // socket -> (pool ife index, ingress slot); bps entries use pool -1
    std::map<int, std::pair<int, int>> placement;
};

Packing tryPack(const FleetState& base, const std::vector<const IspSensorRequest*>& pooled, bool allowBps) {
    Packing packing;
    FleetState state = base;
    std::vector<const IspSensorRequest*> toBps;
    std::vector<const IspSensorRequest*> toPool = pooled;

    while(true) {
        // LPT: heaviest first into the least-loaded open pool.
        bool fits = true;
        FleetState attempt = state;
        std::map<int, std::pair<int, int>> placement;
        for(const auto* req : toPool) {
            int best = -1;
            for(int n = 0; n < kNumPoolIfe; n++) {
                if(attempt.ifeDedicated[n]) {
                    continue;
                }
                if(best < 0 || attempt.poolLoad[n] < attempt.poolLoad[best]) {
                    best = n;
                }
            }
            if(best < 0 || attempt.poolLoad[best] + sensorLoad(*req) > kPoolBudgetPixPerSec) {
                fits = false;
                break;
            }
            attempt.poolLoad[best] += sensorLoad(*req);
            attempt.ifePool[best] = true;
            int slot = -1;
            for(size_t s = 0; s < kIngressNames.size(); s++) {
                if(attempt.ingressAvailable(static_cast<int>(s))) {
                    slot = static_cast<int>(s);
                    break;
                }
            }
            if(slot < 0) {
                fits = false;
                break;
            }
            attempt.ingressUsed[slot] = true;
            placement[req->socket] = {best, slot};
        }
        if(fits) {
            for(const auto* req : toBps) {
                int slot = -1;
                for(size_t s = 0; s < kIngressNames.size(); s++) {
                    if(attempt.ingressAvailable(static_cast<int>(s))) {
                        slot = static_cast<int>(s);
                        break;
                    }
                }
                if(slot < 0 || attempt.bpsLoad + sensorLoad(*req) > kBpsBudgetPixPerSec) {
                    fits = false;
                    break;
                }
                attempt.ingressUsed[slot] = true;
                attempt.bpsLoad += sensorLoad(*req);
                placement[req->socket] = {-1, slot};
            }
        }
        if(fits) {
            packing.ok = true;
            packing.placement = std::move(placement);
            return packing;
        }
        // Pools cannot take the set: spill the lightest sensor to BPS and retry.
        if(!allowBps || toPool.empty()) {
            return packing;
        }
        toBps.push_back(toPool.back());
        toPool.pop_back();
    }
}

constexpr uint32_t kIspRouteMagic = 0x4C580000;
constexpr uint32_t kIspRouteFullIfeMask = 0x7;
constexpr uint32_t kIspRouteInputShift = 3;
constexpr uint32_t kIspRouteProcessorShift = 6;
constexpr uint32_t kIspProcessorBps = 3;

}  // namespace

std::optional<int32_t> encodeFullIfeRoute(const std::string& value) {
    uint32_t route = kIspRouteMagic;
    bool anyIfe = false;
    for(const auto& token : splitTokens(value)) {
        const std::string normalized = normalizeToken(token);
        if(normalized.size() != 1 || normalized[0] < '0' || normalized[0] > '2') {
            spdlog::error("Invalid beta-full-ife entry '{}', expected IFE indices 0..2", token);
            return std::nullopt;
        }
        route |= 1U << static_cast<uint32_t>(normalized[0] - '0');
        anyIfe = true;
    }
    if(!anyIfe) {
        spdlog::error("beta-full-ife control has no IFE indices");
        return std::nullopt;
    }
    return static_cast<int32_t>(route);
}

std::optional<int32_t> encodeSharedIspRoute(const std::string& value) {
    static const std::map<std::string, uint32_t> inputs = {{"sfe0", 0}, {"sfe1", 1}, {"sfe2", 2}, {"ifelite0", 3}, {"ifelite1", 4}};
    static const std::map<std::string, uint32_t> processors = {
        {"ife0", 0}, {"ife1", 1}, {"ife2", 2}, {"bps", kIspProcessorBps}, {"bsp", kIspProcessorBps}};

    const auto tokens = splitTokens(value);
    if(tokens.size() < 2) {
        spdlog::error("beta-shared-isp '{}' needs one ingress and at least one processor, e.g. sfe0+ife0", value);
        return std::nullopt;
    }
    const auto input = inputs.find(normalizeToken(tokens[0]));
    if(input == inputs.end()) {
        spdlog::error("beta-shared-isp ingress '{}' must be sfe0..2 or ife-lite0..1", tokens[0]);
        return std::nullopt;
    }
    uint32_t route = kIspRouteMagic;
    route |= (input->second + 1U) << kIspRouteInputShift;
    for(size_t i = 1; i < tokens.size(); i++) {
        const auto processor = processors.find(normalizeToken(tokens[i]));
        if(processor == processors.end()) {
            spdlog::error("beta-shared-isp processor '{}' must be ife0..2 or bps", tokens[i]);
            return std::nullopt;
        }
        route |= 1U << (kIspRouteProcessorShift + processor->second);
    }
    return static_cast<int32_t>(route);
}

bool ispRouteUsesBps(int32_t route) {
    constexpr uint32_t bpsBit = 1U << (kIspRouteProcessorShift + kIspProcessorBps);
    return (static_cast<uint32_t>(route) & bpsBit) != 0;
}

bool ispRouteHasDedicatedFullIfe(int32_t route) {
    return (static_cast<uint32_t>(route) & kIspRouteFullIfeMask) != 0;
}

std::map<int, IspRouteDecision> IspRouteAllocator::allocate(std::vector<IspSensorRequest> fleet) {
    std::map<int, IspRouteDecision> decisions;
    FleetState state;
    std::vector<IspSensorRequest*> autos;
    bool anyManual = false;
    bool anyWide = false;

    // Pin manual routes first: they consume their resources verbatim.
    for(auto& req : fleet) {
        if(!req.manualFullIfe.empty() || !req.manualSharedIsp.empty()) {
            anyManual = true;
            decisions[req.socket] = manualDecision(req);
            if(!req.manualFullIfe.empty()) {
                for(const auto& token : splitTokens(req.manualFullIfe)) {
                    const std::string normalized = normalizeToken(token);
                    if(normalized.size() == 1 && normalized[0] >= '0' && normalized[0] <= '2') {
                        state.ifeDedicated[normalized[0] - '0'] = true;
                    }
                }
            } else {
                const auto parsed = parseSharedIsp(req.manualSharedIsp);
                if(parsed.ingress >= 0) {
                    state.ingressUsed[parsed.ingress] = true;
                }
                for(const int ife : parsed.ifes) {
                    if(ife < kNumPoolIfe) {
                        state.ifePool[ife] = true;
                        state.poolLoad[ife] += sensorLoad(req);
                    }
                }
                if(parsed.bps) {
                    state.bpsLoad += sensorLoad(req);
                }
            }
        } else {
            autos.push_back(&req);
            if(req.width > kMaxSingleIfeWidth) {
                anyWide = true;
            }
        }
    }

    // Deterministic order: heaviest first, socket index breaking ties.
    std::sort(autos.begin(), autos.end(), [](const IspSensorRequest* a, const IspSensorRequest* b) {
        const double loadA = sensorLoad(*a);
        const double loadB = sensorLoad(*b);
        if(loadA != loadB) {
            return loadA > loadB;
        }
        return a->socket < b->socket;
    });

    // With no manual routes, no wide sensors, and one free full IFE per
    // camera, keep the legacy realtime path: the camera stack picks the
    // physical IFE itself and nothing needs coordinating.
    if(!anyManual && !anyWide && static_cast<int>(autos.size()) <= kNumFullIfe) {
        for(const auto* req : autos) {
            IspRouteDecision decision;
            decision.automatic = true;
            decision.canonical = "realtime";
            decisions[req->socket] = decision;
        }
        return decisions;
    }

    auto fail = [&](const std::string& error) {
        for(const auto* req : autos) {
            if(decisions.count(req->socket) == 0) {
                IspRouteDecision decision;
                decision.automatic = true;
                decision.error = error;
                decisions[req->socket] = decision;
            }
        }
        return decisions;
    };

    // Wide sensors first: they have the largest regret. A free dual-IFE pair
    // beats BPS (no RAW round-trip); the pairing prefers to include IFE2 so a
    // pool-capable IFE survives.
    std::vector<IspSensorRequest*> normals;
    for(auto* req : autos) {
        if(req->width <= kMaxSingleIfeWidth) {
            normals.push_back(req);
            continue;
        }
        static const std::array<std::pair<int, int>, 3> pairs = {{{1, 2}, {0, 2}, {0, 1}}};
        int left = -1;
        int right = -1;
        for(const auto& pair : pairs) {
            if(state.ifeFree(pair.first) && state.ifeFree(pair.second)) {
                left = pair.first;
                right = pair.second;
                break;
            }
        }
        IspRouteDecision decision;
        decision.automatic = true;
        if(left >= 0) {
            state.ifeDedicated[left] = true;
            state.ifeDedicated[right] = true;
            decision.fullIfe = std::to_string(left) + "," + std::to_string(right);
            decision.canonical = canonicalFullIfe({left, right});
        } else {
            int slot = -1;
            for(size_t s = 0; s < kIngressNames.size(); s++) {
                if(state.ingressAvailable(static_cast<int>(s))) {
                    slot = static_cast<int>(s);
                    break;
                }
            }
            if(slot < 0 || state.bpsLoad + sensorLoad(*req) > kBpsBudgetPixPerSec) {
                return fail("no dual-IFE pair or BPS capacity for wide sensor");
            }
            state.ingressUsed[slot] = true;
            state.bpsLoad += sensorLoad(*req);
            decision.sharedIsp = std::string(kIngressNames[slot]) + "+bps";
            decision.canonical = decision.sharedIsp;
        }
        decisions[req->socket] = decision;
    }

    // Dedication is a luxury, granted only while everyone left still fits the
    // pools: try the largest number of dedicated sensors first and back off.
    // IFE2 is handed out first (it can never host a pool), IFE0 last (its
    // dedication also blocks sfe0 ingress).
    std::vector<int> dedicationOrder;
    for(const int n : {2, 1, 0}) {
        if(state.ifeFree(n)) {
            dedicationOrder.push_back(n);
        }
    }

    // First pass keeps everyone off BPS; only if no dedication level packs
    // the pools does the second pass allow spilling the lightest sensors.
    const int maxDedicated = std::min(static_cast<int>(dedicationOrder.size()), static_cast<int>(normals.size()));
    for(const bool allowBps : {false, true}) {
    for(int k = maxDedicated; k >= 0; k--) {
        FleetState attempt = state;
        for(int i = 0; i < k; i++) {
            attempt.ifeDedicated[dedicationOrder[i]] = true;
        }
        std::vector<const IspSensorRequest*> pooled(normals.begin() + k, normals.end());
        const Packing packing = tryPack(attempt, pooled, allowBps);
        if(!packing.ok) {
            continue;
        }
        for(int i = 0; i < k; i++) {
            IspRouteDecision decision;
            decision.automatic = true;
            decision.fullIfe = std::to_string(dedicationOrder[i]);
            decision.canonical = canonicalFullIfe({dedicationOrder[i]});
            decisions[normals[i]->socket] = decision;
        }
        for(const auto* req : pooled) {
            const auto& place = packing.placement.at(req->socket);
            IspRouteDecision decision;
            decision.automatic = true;
            if(place.first >= 0) {
                decision.sharedIsp = std::string(kIngressNames[place.second]) + "+ife" + std::to_string(place.first);
            } else {
                decision.sharedIsp = std::string(kIngressNames[place.second]) + "+bps";
            }
            decision.canonical = decision.sharedIsp;
            decisions[req->socket] = decision;
        }
        return decisions;
    }
    }

    return fail("fleet exceeds ISP capacity (pools and BPS full)");
}

IspRouteAllocator& IspRouteAllocator::getInstance() {
    static IspRouteAllocator instance;
    return instance;
}

void IspRouteAllocator::registerSensor(const IspSensorRequest& req) {
    std::lock_guard<std::mutex> lock(mtx);
    sensors[req.socket] = req;
    generation++;
}

void IspRouteAllocator::unregisterSensor(int socket) {
    std::lock_guard<std::mutex> lock(mtx);
    sensors.erase(socket);
    generation++;
}

IspRouteDecision IspRouteAllocator::resolve(int socket) {
    std::lock_guard<std::mutex> lock(mtx);
    if(decidedGeneration != generation) {
        std::vector<IspSensorRequest> fleet;
        fleet.reserve(sensors.size());
        for(const auto& pair : sensors) {
            fleet.push_back(pair.second);
        }
        decisions = allocate(std::move(fleet));
        decidedGeneration = generation;
    }
    const auto it = decisions.find(socket);
    if(it == decisions.end()) {
        IspRouteDecision decision;
        decision.automatic = true;
        decision.error = "socket was never registered with the ISP route allocator";
        return decision;
    }
    return it->second;
}

}  // namespace dai
