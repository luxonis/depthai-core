#include "utilities/EvaDataMemory.hpp"

#include <algorithm>
#include <cstring>
#include <stdexcept>

#include "utilities/DmaBufMemory.hpp"
#include "utilities/ErrorMacros.hpp"
#include "vendor/DeviceError.hpp"

namespace dai {

namespace {
constexpr const char* UNIT = "Accelerator memory";
}

EvaDataMemory::EvaDataMemory(Private, dai_device_mem* mem, std::size_t size, std::shared_ptr<const void> keepAlive)
    : mem(mem), keepAlive(std::move(keepAlive)), logicalSize(size) {
    dai_device_buf buf{};
    std::size_t offset = 0;
    const dai_status status = dai_device_mem_get(mem, &buf, &offset);
    if(status != DAI_STATUS_OK) {
        dai_device_mem_release(mem);
        this->mem = nullptr;
        device::throwDeviceError(status, UNIT, "dai_device_mem_get", nullptr);
    }
    base = buf.data;
    total = buf.size;
    currentOffset = offset;
    bufferFd = buf.fd;
}

EvaDataMemory::~EvaDataMemory() {
    dai_device_mem_release(mem);
}

std::uint8_t* EvaDataMemory::data() {
    return base + currentOffset;
}

const std::uint8_t* EvaDataMemory::data() const {
    return base + currentOffset;
}

std::size_t EvaDataMemory::size() const {
    return logicalSize - currentOffset;
}

int EvaDataMemory::fd() const {
    return bufferFd;
}

span<std::uint8_t> EvaDataMemory::getData() {
    return {base + currentOffset, logicalSize - currentOffset};
}

span<const std::uint8_t> EvaDataMemory::getData() const {
    return {base + currentOffset, logicalSize - currentOffset};
}

std::size_t EvaDataMemory::getMaxSize() const {
    return total;
}

std::size_t EvaDataMemory::getOffset() const {
    return currentOffset;
}

void EvaDataMemory::setSize(std::size_t newSize) {
    if(newSize > getMaxSize()) {
        throw std::invalid_argument("Cannot set size larger than max size");
    }
    logicalSize = newSize;
}

void EvaDataMemory::setOffset(std::size_t offset) {
    DAI_CHECK_DEVICE(UNIT, dai_device_mem_add_offset(mem, offset));
    currentOffset = std::min(currentOffset + offset, total);
}

span<std::uint8_t> EvaDataMemory::getOffsetData() {
    return getData();
}

span<const std::uint8_t> EvaDataMemory::getOffsetData() const {
    return getData();
}

std::shared_ptr<OffsetMemory> EvaDataMemory::offset(std::size_t offset) {
    dai_device_mem* clone = nullptr;
    DAI_CHECK_DEVICE(UNIT, dai_device_mem_clone(mem, &clone));
    // The view keeps this buffer alive: it uses the same memory
    auto view = std::make_shared<EvaDataMemory>(Private{}, clone, logicalSize, shared_from_this());
    view->setOffset(offset);
    return view;
}

std::shared_ptr<EvaDataMemory> EvaDataMemory::create(std::size_t size) {
    dai_device_mem* mem = nullptr;
    DAI_CHECK_DEVICE(UNIT, dai_device_mem_alloc(size, &mem));
    return std::make_shared<EvaDataMemory>(Private{}, mem, size, nullptr);
}

std::shared_ptr<EvaDataMemory> EvaDataMemory::convertZeroCopy(std::shared_ptr<Memory> data) {
    if(auto evaBuf = std::dynamic_pointer_cast<EvaDataMemory>(data); evaBuf != nullptr) {
        return evaBuf;
    }
    // DMA-BUF memory of the firmware (camera HAL, DMA heap, GStreamer fd memory): register its fd. The buffer keeps
    // `data` alive while the accelerator uses it.
    if(auto dmaBuf = std::dynamic_pointer_cast<dai::gate::DmaBufMemory>(data); dmaBuf != nullptr) {
        const int fd = dmaBuf->dmaBufFd();
        if(fd != -1) {
            auto bytes = data->getData();
            dai_device_mem* mem = nullptr;
            DAI_CHECK_DEVICE(UNIT, dai_device_mem_import(fd, bytes.data(), bytes.size(), &mem));
            return std::make_shared<EvaDataMemory>(Private{}, mem, bytes.size(), std::move(data));
        }
    }
    return nullptr;
}

std::shared_ptr<EvaDataMemory> EvaDataMemory::convert(std::shared_ptr<Memory> data, std::size_t size) {
    std::shared_ptr<EvaDataMemory> evaBuf = convertZeroCopy(data);
    if(evaBuf == nullptr) {
        // Not DMA-BUF memory: copy it into a new accelerator buffer
        size = size ? size : data->getData().size();
        evaBuf = create(size);
        memcpy(evaBuf->getData().data(), data->getData().data(), std::min(size, data->getData().size()));
    }
    return evaBuf;
}

}  // namespace dai
