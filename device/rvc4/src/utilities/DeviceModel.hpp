#pragma once
#include <cstdint>
#include <optional>

namespace dai {
namespace gate {

class DeviceModel {
   public:
    enum class Model { UNKNOWN_MODEL, OAK4_D, OAK4_S, DEV_KIT, OAK4_LITE, OAK4_FFC, OAK4_MAX, FAST_STATION_CAMERA, VISUAL_CORTEX };
    static Model get();
    static std::optional<std::uint32_t> getRevision();
};

}  // namespace gate
}  // namespace dai
