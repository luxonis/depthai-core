#include <iostream>

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

int main() {
    // Create pipeline
    dai::Pipeline pipeline;

    // Define sources and outputs
    auto monoLeft = pipeline.create<dai::node::MonoCamera>();
    auto monoRight = pipeline.create<dai::node::MonoCamera>();
    auto color = pipeline.create<dai::node::ColorCamera>();
    auto stereo = pipeline.create<dai::node::StereoDepth>();
    auto sync = pipeline.create<dai::node::Sync>();

    auto xoutGrp = pipeline.create<dai::node::XLinkOut>();

    // XLinkOut
    xoutGrp->setStreamName("xout");

    // Properties
    monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    monoLeft->setCamera("left");
    monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    monoRight->setCamera("right");

    stereo->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::HIGH_ACCURACY);

    color->setCamera("color");

    sync->setSyncThresholdMs(100);

    // Linking
    monoLeft->out.link(stereo->left);
    monoRight->out.link(stereo->right);

    stereo->depth.link(sync->inputs["depth"]);
    color->video.link(sync->inputs["video"]);

    sync->out.link(xoutGrp->input);

    // Connect to device and start pipeline
    dai::Device device(pipeline);

    auto queue = device.getOutputQueue("xout", 10, true);

    while(true) {
        auto msgGrp = queue->get<dai::MessageGroup>();
        std::cout << "Got message group" << std::endl;
        for(auto& frm : *msgGrp) {
            auto imgFrm = std::dynamic_pointer_cast<dai::ImgFrame>(frm.second);
            cv::imshow(frm.first, imgFrm->getFrame());
        }

        int key = cv::waitKey(1);
        if(key == 'q' || key == 'Q') {
            return 0;
        }
    }
    return 0;
}
