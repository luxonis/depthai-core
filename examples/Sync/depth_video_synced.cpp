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

    sync->setSyncThreshold(std::chrono::milliseconds(100));

    // Linking
    monoLeft->out.link(stereo->left);
    monoRight->out.link(stereo->right);

    stereo->disparity.link(sync->inputs["disparity"]);
    color->video.link(sync->inputs["video"]);

    sync->out.link(xoutGrp->input);

    // Connect to device and start pipeline
    dai::Device device(pipeline);

    auto queue = device.getOutputQueue("xout", 10, true);

    float disparityMultiplier = 255 / stereo->initialConfig.getMaxDisparity();

    while(true) {
        auto msgGrp = queue->get<dai::MessageGroup>();
        for(auto& frm : *msgGrp) {
            auto imgFrm = std::dynamic_pointer_cast<dai::ImgFrame>(frm.second);
            cv::Mat img = imgFrm->getCvFrame();
            if(frm.first == "disparity") {
                img.convertTo(img, CV_8UC1, disparityMultiplier);  // Extend disparity range
                cv::applyColorMap(img, img, cv::COLORMAP_JET);
            }
            cv::imshow(frm.first, img);
        }

        int key = cv::waitKey(1);
        if(key == 'q' || key == 'Q') {
            return 0;
        }
    }
    return 0;
}
