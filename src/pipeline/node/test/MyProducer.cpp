#include "depthai/pipeline/node/test/MyProducer.hpp"

// std
#include <chrono>

// project
#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {
namespace node {
namespace test {


void MyProducer::run() {
    std::cout << "Hello, I just woke up and I'm ready to do some work!\n";
    while(isRunning()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1500));

        auto buf = std::make_shared<Buffer>();
        buf->setData({1, 2, 3});

        std::cout << "sending message (ptr: " << buf.get() << "\n";
        out.send(buf);
    }
}

}  // namespace test
}  // namespace node
}  // namespace dai
