#include "depthai/utility/SharedMemory.hpp"

namespace dai {

SharedMemory::~SharedMemory() {
    unmapFd();
#if defined(__unix__) && !defined(__APPLE__)
    if(fd > 0) {
        close(fd);
    }
#endif
}
}  // namespace dai