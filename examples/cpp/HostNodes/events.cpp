
#include <chrono>
#include <iostream>
#include <string>

#include "depthai/depthai.hpp"
#include "depthai/utility/EventsManager.hpp"

int main(int argc, char* argv[]) {
    dai::Pipeline pipeline(true);

    auto eventsManager = std::make_shared<dai::utility::EventsManager>();
    eventsManager->setUrl("https://events-ingest.apps.stg.hubcloud");
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
    auto fileData2 = std::make_shared<dai::utility::EventData>("/test.txt");
    std::vector<std::shared_ptr<dai::utility::EventData>> data2;
    data2.push_back(fileData2);
    // will fail, you sendEvent instead of sendSnap
    eventsManager->sendSnap("testdata2", nullptr, data2, {"tag5", "tag6"}, {{"key8", "value8"}});
    auto fileData3 = std::make_shared<dai::utility::EventData>("/test.jpg");
    std::vector<std::shared_ptr<dai::utility::EventData>> data3;
    data3.push_back(fileData3);
    eventsManager->sendSnap("testdata3", nullptr, data3, {"tag7", "tag8"}, {{"key8", "value8"}});
    std::vector<std::shared_ptr<dai::utility::EventData>> data4;
    data4.push_back(fileData);
    data4.push_back(fileData2);
    eventsManager->sendEvent("testdata4", nullptr, data4, {"tag9", "tag10"}, {{"key8", "value8"}});
    data4.push_back(fileData3);
    while(pipeline.isRunning()) {
        auto rgb = previewQ->get<dai::ImgFrame>();

        // Do something with the data
        // ...

        if(!sent) {
            eventsManager->sendSnap("rgb", rgb, {}, {"tag11", "tag12"}, {{"key", "value"}});
            // will fail due to two images being sent, use sendEvent instead
            eventsManager->sendSnap("test2", rgb, data3, {"tag13", "tag14"}, {{"key8", "value8"}});
            // will fail, sendSnap requires only one image data to be present
            eventsManager->sendSnap("test3", rgb, data4, {"tag13", "tag14"}, {{"key8", "value8"}});
            sent = true;
        }
        //
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        // eventsManager->sendSnap("test8", {{"key8", "value8"}}, {"tag15", "tag16"});
    }

    return EXIT_SUCCESS;
}
