#include <chrono>
#include <iostream>
#include <string>

#include "depthai/depthai.hpp"
#include "depthai/utility/EventsManager.hpp"

int main(int argc, char* argv[]) {
    dai::Pipeline pipeline(true);

    auto eventsManager = std::make_shared<dai::utility::EventsManager>();

    // Enter your hubs api-key
    eventsManager->setUrl("https://events.cloud-stg.luxonis.com");
    eventsManager->setToken("");

    // Color camera node
    auto camRgb = pipeline.create<dai::node::Camera>()->build();
    auto* preview = camRgb->requestOutput(std::make_pair(256, 256));

    auto previewQ = preview->createOutputQueue();

    pipeline.start();

    std::vector<std::shared_ptr<dai::utility::FileData>> data;

    while(pipeline.isRunning()) {
        auto rgb = previewQ->get<dai::ImgFrame>();

        std::string str = "image_";
        std::stringstream ss;
        ss << str << data.size();

        auto rgbData = std::make_shared<dai::utility::FileData>(rgb, ss.str());
        data.emplace_back(rgbData);

        if (data.size() == 5)
        {
            eventsManager->sendSnap("ImgFrame", {"EventsExample", "C++"}, {{"key_0", "value_0"}, {"key_1", "value_1"}}, "", data);
            data.clear();
            std::this_thread::sleep_for(std::chrono::milliseconds(3000));
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(400));
    }

    return EXIT_SUCCESS;
}
