#include "depthai/pipeline/node/test/MyProducer.hpp"

// std
#include <chrono>

// project
#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {
namespace node {
namespace test {

MyProducer::MyProducer(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId) : MyProducer(par, nodeId, std::make_unique<MyProducer::Properties>()) {}
MyProducer::MyProducer(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId, std::unique_ptr<Properties> props)
    : NodeCRTP<ThreadedNode, MyProducer, XLinkOutProperties>(par, nodeId, std::move(props)) {
    setOutputRefs(&out);
}


void MyProducer::run() {
    std::cout << "Hello, I just woke up and I'm ready to do some work!\n";
    while(isRunning()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1500));

        auto buf = std::make_shared<Buffer>();
        buf->setData({1,2,3});

        std::cout << "sending message (ptr: " << buf.get() << ", raw: " << buf->getRaw().get() << ")\n";
        out.send(buf);
    }
}


}  // namespace test
}  // namespace node
}  // namespace dai
