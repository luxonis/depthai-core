#include <depthai/depthai.hpp>
#include <opencv2/opencv.hpp>

constexpr int SHAPE = 300;

int main() {
    dai::Pipeline p;

    auto camRgb = p.create<dai::node::ColorCamera>();
    auto left = p.create<dai::node::MonoCamera>();
    auto right = p.create<dai::node::MonoCamera>();
    auto manipLeft = p.create<dai::node::ImageManip>();
    auto manipRight = p.create<dai::node::ImageManip>();
    auto nn = p.create<dai::node::NeuralNetwork>();
    auto cast = p.create<dai::node::Cast>();
    auto castXout = p.create<dai::node::XLinkOut>();

    camRgb->setPreviewSize(SHAPE, SHAPE);
    camRgb->setInterleaved(false);
    camRgb->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);

    left->setCamera("left");
    left->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    manipLeft->initialConfig.setResize(SHAPE, SHAPE);
    manipLeft->initialConfig.setFrameType(dai::ImgFrame::Type::BGR888p);

    right->setCamera("right");
    right->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    manipRight->initialConfig.setResize(SHAPE, SHAPE);
    manipRight->initialConfig.setFrameType(dai::ImgFrame::Type::BGR888p);

    nn->setBlobPath(BLOB_PATH);
    nn->setNumInferenceThreads(2);

    castXout->setStreamName("cast");
    cast->setOutputFrameType(dai::ImgFrame::Type::BGR888p);

    // Linking
    left->out.link(manipLeft->inputImage);
    right->out.link(manipRight->inputImage);
    manipLeft->out.link(nn->inputs["img1"]);
    camRgb->preview.link(nn->inputs["img2"]);
    manipRight->out.link(nn->inputs["img3"]);
    nn->out.link(cast->input);
    cast->output.link(castXout->input);

    // Pipeline is defined, now we can connect to the device
    dai::Device device(p);
    auto qCast = device.getOutputQueue("cast", 4, false);

    while(true) {
        auto inCast = qCast->get<dai::ImgFrame>();
        if(inCast) {
            cv::imshow("Concated frames", inCast->getCvFrame());
        }

        if(cv::waitKey(1) == 'q') {
            break;
        }
    }

    return 0;
}