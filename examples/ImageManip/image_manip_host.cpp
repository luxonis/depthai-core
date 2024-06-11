#include <opencv2/core/mat.hpp>
#include "depthai/depthai.hpp"
#include "depthai/pipeline/InputQueue.hpp"
#include "depthai/pipeline/datatype/ImageManipConfig.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/node/ImageManip.hpp"

using namespace dai::impl;
int main(int argc, char** argv) {
    if(argc <= 1) return -1;
    cv::Mat img = cv::imread(argv[1]);
    ImageManipOperations manip;
    dai::ImageManipBase base;
    base.rotateDegrees(1, false);
    manip.build(base, img.cols, img.rows, 3);
    const auto [w, h, c] = manip.getOutputSize();
    auto frame = std::make_shared<dai::ImgFrame>();
    frame->setCvFrame(img, dai::ImgFrame::Type::RGB888i);
    std::vector<uint8_t> dst(w * h * c);
    manip.apply(frame, dst);
    cv::Mat out(h, w, CV_8UC3, dst.data());
    cv::imshow("in", img);
    cv::imshow("out", out);
    cv::waitKey(0);
    /*auto device = std::make_shared<dai::Device>("10.12.103.55");*/
    /*dai::Pipeline pipeline(device);*/
    /**/
    /*auto manip = pipeline.create<dai::node::ImageManip>();*/
    /**/
    /*manip->setMaxOutputFrameSize(4000000);*/
    /*manip->initialConfig.base.rotateDegrees(-2, false);*/
    /*/*manip->initialConfig.setResizeThumbnail(300, 300);*/
    /**/
    /*auto inputQ = manip->inputImage.createInputQueue();*/
    /*auto outputQ = manip->out.createOutputQueue();*/
    /**/
    /*int numImages = argc - 1;*/
    /**/
    /*try {*/
    /*    int index = 0;*/
    /*    pipeline.start();*/
    /*    while(true) {*/
    /*        cv::Mat img = cv::imread(argv[index + 1]);*/
    /*        index = ++index % numImages;*/
    /*        if(img.empty()) continue;*/
    /*        std::cout << "Sending image..." << std::endl;*/
    /*        auto frame = std::make_shared<dai::ImgFrame>();*/
    /*        frame->setCvFrame(img, dai::ImgFrame::Type::RGB888i);*/
    /*        inputQ->send(frame);*/
    /*        auto imgFrame = outputQ->get<dai::ImgFrame>();*/
    /*        std::cout << "Got image..." << std::endl;*/
    /*        cv::imshow("input", frame->getCvFrame());*/
    /**/
    /*        cv::imshow("manip", imgFrame->getCvFrame());*/
    /**/
    /*        cv::waitKey(10);*/
    /*    }*/
    /*} catch(const std::exception& e) {*/
    /*    std::cerr << "Error: " << e.what() << std::endl;*/
    /*}*/
    /*pipeline.stop();*/
}

namespace dai {


}  // namespace dai
