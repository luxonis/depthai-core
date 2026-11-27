#pragma once

#include <cstdint>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace dai {

// Explicit ISP routes travel to the camera stack as an INT32 (through the
// offline front end session tag, DAI_CAMERA_TAG_OFFLINE_FRONT_END): a magic
// tag plus bits of dedicated full image front ends, or an ingress selector plus
// shared-processor bits. These helpers are the single encoder for the route
// strings used in decisions and misc controls ("beta-full-ife" = image front
// end indices like "2" or "0,1"; "beta-shared-isp" = ingress+processors like
// "sfe0+ife0" or "ife-lite1+bps").
std::optional<int32_t> encodeFullIfeRoute(const std::string& value);
std::optional<int32_t> encodeSharedIspRoute(const std::string& value);
bool ispRouteUsesBps(int32_t route);
bool ispRouteHasDedicatedFullIfe(int32_t route);

// One camera's request for ISP resources, registered before any camera opens.
struct IspSensorRequest {
    int socket{-1};
    std::string socketName;
    uint32_t width{0};
    uint32_t height{0};
    float fps{0.0f};
    // Manual "beta-full-ife" / "beta-shared-isp" misc controls (empty = automatic)
    std::string manualFullIfe;
    std::string manualSharedIsp;
};

// The allocator's verdict for one camera. Exactly one of fullIfe / sharedIsp is
// non-empty for an explicit route; both empty selects the legacy realtime path
// where the camera stack picks the physical image front end itself.
struct IspRouteDecision {
    std::string fullIfe;
    std::string sharedIsp;
    bool automatic{false};
    // Stable human/test-readable form: "realtime", "full-ife:2", "sfe0+ife0",
    // "ife-lite0+bps", "full-ife:1,2"
    std::string canonical;
    std::string error;
};

// Fleet-wide automatic selection of ISP routes (dedicated full image front
// ends, shared offline front end pools with their ingress, spill to the Bayer
// processing stage) with manual routes pinned first. Cameras register at construction (all camera nodes are constructed
// before any of them opens) and resolve at open time, so the first resolve
// sees the whole fleet.
class IspRouteAllocator {
   public:
    static IspRouteAllocator& getInstance();

    void registerSensor(const IspSensorRequest& req);
    void unregisterSensor(int socket);
    IspRouteDecision resolve(int socket);

    // Pure allocation logic, deterministic; exposed for testing.
    static std::map<int, IspRouteDecision> allocate(std::vector<IspSensorRequest> fleet);

   private:
    IspRouteAllocator() = default;

    std::mutex mtx;
    std::map<int, IspSensorRequest> sensors;
    std::map<int, IspRouteDecision> decisions;
    uint64_t generation{0};
    uint64_t decidedGeneration{~0ULL};
};

}  // namespace dai
