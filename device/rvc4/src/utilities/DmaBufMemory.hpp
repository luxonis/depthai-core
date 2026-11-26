#pragma once

namespace dai {
namespace gate {

class DmaBufMemory {
   public:
    virtual ~DmaBufMemory() = default;

    /// DMA-BUF file descriptor of the memory, or -1 when the memory has none.
    virtual int dmaBufFd() const = 0;
};

}  // namespace gate
}  // namespace dai
