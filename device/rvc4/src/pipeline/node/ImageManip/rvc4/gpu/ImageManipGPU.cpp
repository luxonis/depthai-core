#include "ImageManipGPU.hpp"

#include <iostream>
#include <limits>
#include <sstream>

#include "camera/HalBufferMemory.hpp"
#include "utilities/DmaHeapMemory.hpp"
#include "utilities/GstBufferMemory.hpp"

namespace dai::gate::utility {

#if defined(CL_MEM_EXT_HOST_PTR_QCOM) && (defined(CL_MEM_ION_HOST_PTR_QCOM) || defined(CL_MEM_DMABUF_HOST_PTR_QCOM))
std::optional<ExternalMemoryInfo> getExternalMemoryInfo(const std::shared_ptr<OffsetMemory>& memory) {
    if(memory == nullptr) {
        return std::nullopt;
    }

    ExternalMemoryInfo info;
    info.hostPtr = memory->getData().data();
    if(info.hostPtr == nullptr) {
        return std::nullopt;
    }

    if(auto dmaHeapMemory = ConvertedOffsetMemory::cast<DmaHeapMemory>(memory); dmaHeapMemory != nullptr) {
        info.dmaBufFd = dmaHeapMemory->fd();
        info.allocationOwner = dmaHeapMemory->getAllocationOwner();
        return info.dmaBufFd >= 0 ? std::optional<ExternalMemoryInfo>(info) : std::nullopt;
    }

    if(auto halBufferMemory = ConvertedOffsetMemory::cast<HalBufferMemory>(memory); halBufferMemory != nullptr) {
        if(halBufferMemory->privHnd == nullptr) {
            return std::nullopt;
        }
        info.dmaBufFd = halBufferMemory->privHnd->fd;
        info.allocationOwner = halBufferMemory;
        return info.dmaBufFd >= 0 ? std::optional<ExternalMemoryInfo>(info) : std::nullopt;
    }

    if(auto gstMemory = ConvertedOffsetMemory::cast<GstBufferMemory>(memory); gstMemory != nullptr) {
        if(gstMemory->map.memory == nullptr || !gst_is_fd_memory(gstMemory->map.memory)) {
            return std::nullopt;
        }
        info.dmaBufFd = gst_fd_memory_get_fd(gstMemory->map.memory);
        info.allocationOwner = gstMemory;
        return info.dmaBufFd >= 0 ? std::optional<ExternalMemoryInfo>(info) : std::nullopt;
    }

    return std::nullopt;
}
#endif

void logError(const std::shared_ptr<spdlog::async_logger>& logger, const std::exception& error) {
    if(logger != nullptr) {
        logger->error("{}", error.what());
    } else {
        std::cerr << error.what() << std::endl;
    }
}

void requireInitialized(bool initialized, const char* message) {
    if(!initialized) {
        throw std::runtime_error(message);
    }
}

std::size_t validatePlaneIndex(int plane, std::size_t planeCount, const char* negativeMessage, const char* outOfBoundsMessage) {
    if(plane < 0) {
        throw std::invalid_argument(negativeMessage);
    }
    const auto planeIndex = static_cast<std::size_t>(plane);
    if(planeIndex >= planeCount) {
        throw std::invalid_argument(outOfBoundsMessage);
    }
    return planeIndex;
}

std::size_t getRequiredImageBytes(std::size_t width, std::size_t height, std::size_t strideBytes, std::size_t pixelStrideBytes) {
    if(width == 0 || height == 0) {
        return 0;
    }

    return ((height - 1) * strideBytes) + (width * pixelStrideBytes);
}

std::size_t getMaxBufferSize(cl_device_id device) {
    cl_ulong maxAllocBytes = 0;
    checkStatus(clGetDeviceInfo(device, CL_DEVICE_MAX_MEM_ALLOC_SIZE, sizeof(maxAllocBytes), &maxAllocBytes, nullptr),
                "clGetDeviceInfo(CL_DEVICE_MAX_MEM_ALLOC_SIZE)");
    if(maxAllocBytes > static_cast<cl_ulong>(std::numeric_limits<std::size_t>::max())) {
        return std::numeric_limits<std::size_t>::max();
    }
    return static_cast<std::size_t>(maxAllocBytes);
}

void validateBufferSize(cl_device_id device, std::size_t sizeBytes, const char* what) {
    if(sizeBytes == 0) {
        std::ostringstream stream;
        stream << what << " size is 0 bytes";
        throw ClExecutionError(stream.str());
    }

    const std::size_t maxBufferSize = getMaxBufferSize(device);
    if(sizeBytes > maxBufferSize) {
        std::ostringstream stream;
        stream << what << " too large: requested " << sizeBytes << " bytes, max " << maxBufferSize << " bytes";
        throw ClExecutionError(stream.str());
    }
}

void checkStatus(cl_int status, const char* what) {
    if(status != CL_SUCCESS) {
        std::ostringstream stream;
        stream << what << " failed with OpenCL error " << status;
        throw ClExecutionError(stream.str());
    }
}

std::string getDeviceExtensions(cl_device_id device) {
    size_t extensionsSize = 0;
    checkStatus(clGetDeviceInfo(device, CL_DEVICE_EXTENSIONS, 0, nullptr, &extensionsSize), "clGetDeviceInfo(CL_DEVICE_EXTENSIONS:size)");
    if(extensionsSize == 0) {
        return {};
    }

    std::string extensions(extensionsSize, '\0');
    checkStatus(clGetDeviceInfo(device, CL_DEVICE_EXTENSIONS, extensionsSize, extensions.data(), nullptr), "clGetDeviceInfo(CL_DEVICE_EXTENSIONS)");
    if(!extensions.empty() && extensions.back() == '\0') {
        extensions.pop_back();
    }
    return extensions;
}

bool hasExtensionToken(const std::string& extensionList, const char* extension) {
    if(extension == nullptr || extension[0] == '\0') {
        return false;
    }

    const std::string needle(extension);
    std::size_t pos = 0;
    while((pos = extensionList.find(needle, pos)) != std::string::npos) {
        const bool isTokenStart = pos == 0 || extensionList[pos - 1] == ' ';
        const std::size_t end = pos + needle.size();
        const bool isTokenEnd = end == extensionList.size() || extensionList[end] == ' ';
        if(isTokenStart && isTokenEnd) {
            return true;
        }
        pos = end;
    }

    return false;
}

std::string getBuildLog(cl_program builtProgram, cl_device_id builtDevice) {
    size_t logSize = 0;
    cl_int status = clGetProgramBuildInfo(builtProgram, builtDevice, CL_PROGRAM_BUILD_LOG, 0, nullptr, &logSize);
    if(status != CL_SUCCESS || logSize == 0) {
        return {};
    }

    std::string log(logSize, '\0');
    status = clGetProgramBuildInfo(builtProgram, builtDevice, CL_PROGRAM_BUILD_LOG, logSize, log.data(), nullptr);
    if(status != CL_SUCCESS) {
        return {};
    }
    return log;
}

}  // namespace dai::gate::utility
