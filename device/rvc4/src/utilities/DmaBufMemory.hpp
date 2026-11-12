#pragma once

namespace dai {
namespace gate {

/**
 * Memory backed by a DMA-BUF.
 *
 * The memory types of the firmware that hold DMA-BUF memory (camera HAL buffers, DMA heap buffers,
 * GStreamer fd memory) implement this interface next to dai::Memory. Code that shares buffers with
 * hardware, such as the vision accelerator of the private module, gets the file descriptor here and
 * does not depend on the concrete memory types.
 */
class DmaBufMemory {
   public:
    virtual ~DmaBufMemory() = default;

    /// DMA-BUF file descriptor of the memory, or -1 when the memory has none.
    virtual int dmaBufFd() const = 0;
};

}  // namespace gate
}  // namespace dai
