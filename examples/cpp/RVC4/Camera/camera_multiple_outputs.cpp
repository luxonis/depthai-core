// Includes common necessary includes for development using depthai library
#include <stdexcept>
#include <string>

#include "depthai/capabilities/ImgFrameCapability.hpp"
#include "depthai/depthai.hpp"
#include "depthai/pipeline/node/Sync.hpp"
#include "depthai/pipeline/node/host/Display.hpp"

int main(int argc, char** argv) {
    if(argc < 4 || (argc - 1) % 3 != 0) {
        throw std::runtime_error("USAGE: ./camera_multiple_outputs 1920 1080 0 640 480 1\nWHERE 0 is resize mode: 0 == CROP, 1 == STRETCH, 2 == LETTERBOX");
    }
    std::vector<std::tuple<uint32_t, uint32_t, uint32_t>> sizes;
    for(int index = 1; index < argc - 1; index += 3) {
        sizes.emplace_back(std::stoul(argv[index]), std::stoul(argv[index + 1]), std::stoul(argv[index + 2]));
    }

    dai::Pipeline pipeline;

    auto camRgb = pipeline.create<dai::node::Camera>()->build();
    auto sync = pipeline.create<dai::node::Sync>();
    auto display = pipeline.create<dai::node::Display>();

    sync->setSyncAttempts(0);
    sync->setRunOnHost(true);

    if(sizes.empty()) {
        throw std::runtime_error("internal error to few sizes");
    }

    int index = 0;
    for(const auto& size : sizes) {
        ++index;
        dai::ImgFrameCapability cap;
        cap.type = dai::ImgFrame::Type::NV12;  // Fastest
        cap.size.value = std::pair{std::get<0>(size), std::get<1>(size)};
        auto mode = std::get<2>(size);
        switch(mode) {
            case 0:
                cap.resizeMode = dai::ImgResizeMode::CROP;
                break;
            case 1:
                cap.resizeMode = dai::ImgResizeMode::STRETCH;
                break;
            case 2:
                cap.resizeMode = dai::ImgResizeMode::LETTERBOX;
            default:
                throw std::runtime_error("Resize mode argument (every 3rd) must be 0, 1 or 2");
        }
        auto* output = camRgb->requestOutput(cap, true);
        output->link(sync->inputs[std::to_string(index)]);
    }

    sync->out.link(display->input);

    pipeline.start();

    pipeline.wait();

    return 0;
}
