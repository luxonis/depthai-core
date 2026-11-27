#include "utilities/GpuAvailability.hpp"

#include <fstream>
#include <mutex>

namespace dai::utils {

bool hasGPU() {
    static std::once_flag once;
    static bool cached = false;

    std::call_once(once, []() {
        // Fast/robust check for a real Adreno GPU on these devices: KGSL must be present.
        // Note: /sys/class/kgsl/kgsl-3d0/gpu_model will exist if the Adreno kernel driver is enabled.
        std::ifstream ifs("/sys/class/kgsl/kgsl-3d0/gpu_model");
        cached = ifs.good();
    });

    return cached;
}

}  // namespace dai::utils
