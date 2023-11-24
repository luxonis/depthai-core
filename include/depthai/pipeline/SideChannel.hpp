#pragma once
#include <mutex>

#include "depthai-shared/utility/LockingQueue.hpp"
#include "depthai/pipeline/datatype/ADatatype.hpp"

namespace dai {

class SideChannel {
   public:
    void sendMessage(const std::shared_ptr<dai::ADatatype>& message);
    SideChannel();
    void stop();
    void start();

   private:
    std::thread thread;
    LockingQueue<std::shared_ptr<dai::ADatatype>> lockingQueue{50, false};
    std::atomic_bool running{true};
    void threadedRun();
};

}  // namespace dai