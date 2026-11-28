#include "utilities/GpuAvailability.hpp"

#include <fstream>
#include <mutex>

namespace dai::utils {

bool hasGPU() {
    static std::once_flag once;
    static bool cached = false;

    std::call_once(once, []() {
        std::ifstream ifs("/sys/class/kgsl/kgsl-3d0/gpu_model");  // gpu_model exists only when the Adreno KGSL kernel driver is enabled
        cached = ifs.good();
    });

    return cached;
}

}  // namespace dai::utils
