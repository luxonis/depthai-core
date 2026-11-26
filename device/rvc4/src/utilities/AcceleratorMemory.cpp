#include "utilities/AcceleratorMemory.hpp"

#include <algorithm>
#include <cstring>
#include <stdexcept>

#include "camera/HalBufferMemory.hpp"
#include "utilities/DmaHeapMemory.hpp"
#include "utilities/ErrorMacros.hpp"
#include "utilities/GstBufferMemory.hpp"
#include "vendor/DeviceError.hpp"

namespace dai {

namespace {
constexpr const char* UNIT = "Accelerator memory";
}

AcceleratorMemory::AcceleratorMemory(Private, dai_device_mem* mem, std::size_t size, std::shared_ptr<const void> keepAlive)
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

AcceleratorMemory::~AcceleratorMemory() {
    dai_device_mem_release(mem);
}

std::uint8_t* AcceleratorMemory::data() {
    return base + currentOffset;
}

const std::uint8_t* AcceleratorMemory::data() const {
    return base + currentOffset;
}

std::size_t AcceleratorMemory::size() const {
    return logicalSize - currentOffset;
}

int AcceleratorMemory::fd() const {
    return bufferFd;
}

span<std::uint8_t> AcceleratorMemory::getData() {
    return {base + currentOffset, logicalSize - currentOffset};
}

span<const std::uint8_t> AcceleratorMemory::getData() const {
    return {base + currentOffset, logicalSize - currentOffset};
}

std::size_t AcceleratorMemory::getMaxSize() const {
    return total;
}

std::size_t AcceleratorMemory::getOffset() const {
    return currentOffset;
}

void AcceleratorMemory::setSize(std::size_t newSize) {
    if(newSize > getMaxSize()) {
        throw std::invalid_argument("Cannot set size larger than max size");
    }
    logicalSize = newSize;
}

void AcceleratorMemory::setOffset(std::size_t offset) {
    DAI_CHECK_DEVICE(UNIT, dai_device_mem_add_offset(mem, offset));
    currentOffset = std::min(currentOffset + offset, total);
}

span<std::uint8_t> AcceleratorMemory::getOffsetData() {
    return getData();
}

span<const std::uint8_t> AcceleratorMemory::getOffsetData() const {
    return getData();
}

std::shared_ptr<OffsetMemory> AcceleratorMemory::offset(std::size_t offset) {
    dai_device_mem* clone = nullptr;
    DAI_CHECK_DEVICE(UNIT, dai_device_mem_clone(mem, &clone));
    // The view keeps this buffer alive: it uses the same memory
    auto view = std::make_shared<AcceleratorMemory>(Private{}, clone, logicalSize, shared_from_this());
    view->setOffset(offset);
    return view;
}

std::shared_ptr<AcceleratorMemory> AcceleratorMemory::create(std::size_t size) {
    dai_device_mem* mem = nullptr;
    DAI_CHECK_DEVICE(UNIT, dai_device_mem_alloc(size, &mem));
    return std::make_shared<AcceleratorMemory>(Private{}, mem, size, nullptr);
}

std::shared_ptr<AcceleratorMemory> AcceleratorMemory::convertZeroCopy(std::shared_ptr<Memory> data) {
    if(auto accelBuf = std::dynamic_pointer_cast<AcceleratorMemory>(data); accelBuf != nullptr) {
        return accelBuf;
    }
    // DMA-BUF memory of the firmware (camera HAL, DMA heap, GStreamer fd memory): register its fd. The buffer keeps
    // `data` alive while the accelerator uses it.
    int fd = -1;
    if(auto hal = std::dynamic_pointer_cast<dai::gate::HalBufferMemory>(data); hal != nullptr) {
        fd = hal->privHnd->fd;
    } else if(auto dmaHeap = std::dynamic_pointer_cast<dai::gate::DmaHeapMemory>(data); dmaHeap != nullptr) {
        fd = dmaHeap->fd();
    } else if(auto gst = std::dynamic_pointer_cast<dai::gate::GstBufferMemory>(data); gst != nullptr) {
        fd = gst_fd_memory_get_fd(gst->map.memory);
    }
    if(fd == -1) return nullptr;
    auto bytes = data->getData();
    dai_device_mem* mem = nullptr;
    DAI_CHECK_DEVICE(UNIT, dai_device_mem_import(fd, bytes.data(), bytes.size(), &mem));
    return std::make_shared<AcceleratorMemory>(Private{}, mem, bytes.size(), std::move(data));
}

std::shared_ptr<AcceleratorMemory> AcceleratorMemory::convert(std::shared_ptr<Memory> data, std::size_t size) {
    std::shared_ptr<AcceleratorMemory> accelBuf = convertZeroCopy(data);
    if(accelBuf == nullptr) {
        // Not DMA-BUF memory: copy it into a new accelerator buffer
        size = size ? size : data->getData().size();
        accelBuf = create(size);
        memcpy(accelBuf->getData().data(), data->getData().data(), std::min(size, data->getData().size()));
    }
    return accelBuf;
}

}  // namespace dai
