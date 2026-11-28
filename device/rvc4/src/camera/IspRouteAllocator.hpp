#pragma once

#include <cstdint>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace dai {

std::optional<int32_t> encodeFullIfeRoute(const std::string& value);  // Sent as INT32 in DAI_CAMERA_TAG_OFFLINE_FRONT_END; value: IFE indices like "2" or "0,1"
std::optional<int32_t> encodeSharedIspRoute(const std::string& value);
bool ispRouteUsesBps(int32_t route);
bool ispRouteHasDedicatedFullIfe(int32_t route);

struct IspSensorRequest {
    int socket{-1};
    std::string socketName;
    uint32_t width{0};
    uint32_t height{0};
    float fps{0.0f};
    std::string manualFullIfe;  // Manual "beta-full-ife" / "beta-shared-isp" misc controls (empty = automatic)
    std::string manualSharedIsp;
};

struct IspRouteDecision {  // Exactly one of fullIfe/sharedIsp is set for an explicit route; both empty = legacy realtime path
    std::string fullIfe;
    std::string sharedIsp;
    bool automatic{false};
    std::string canonical;  // Stable form for logs and tests: "realtime", "full-ife:2", "sfe0+ife0", "ife-lite0+bps"
    std::string error;
};

class IspRouteAllocator {  // Register at construction, resolve at open: all cameras are built before any opens
   public:
    static IspRouteAllocator& getInstance();

    void registerSensor(const IspSensorRequest& req);
    void unregisterSensor(int socket);
    IspRouteDecision resolve(int socket);

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
