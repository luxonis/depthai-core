#include <chrono>
#include <iostream>
#include <string>

#include "depthai/depthai.hpp"
#include "depthai/utility/EventsManager.hpp"

int main(int argc, char* argv[]) {
    dai::Pipeline pipeline(true);

    auto eventsManager = std::make_shared<dai::utility::EventsManager>();
    eventsManager->setLogResponse(true);
    // Color camera node
    auto camRgb = pipeline.create<dai::node::Camera>()->build();
    auto* preview = camRgb->requestOutput(std::make_pair(256, 256));

    auto previewQ = preview->createOutputQueue();

    pipeline.start();
    bool sent = false;
    eventsManager->sendEvent("test", nullptr, {}, {"tag1", "tag2"}, {{"key1", "value1"}});

    std::this_thread::sleep_for(std::chrono::milliseconds(7000));

    auto fileData = std::make_shared<dai::utility::EventData>("abc", "test_bin.txt", "text/plain");
    std::vector<std::shared_ptr<dai::utility::EventData>> data;
    data.emplace_back(fileData);
    eventsManager->sendEvent("testdata", nullptr, data, {"tag3", "tag4"}, {{"key8", "value8"}});
    while(pipeline.isRunning()) {
        auto rgb = previewQ->get<dai::ImgFrame>();

        // Do something with the data
        // ...

        if(!sent) {
            eventsManager->sendSnap("rgb", rgb, {}, {"tag11", "tag12"}, {{"key", "value"}});
            sent = true;
        }
        //
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    return EXIT_SUCCESS;
}
