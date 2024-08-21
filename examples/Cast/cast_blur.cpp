#include <depthai/depthai.hpp>
#include <opencv2/opencv.hpp>

constexpr int SHAPE = 300;

int main() {
    dai::Pipeline p;

    auto camRgb = p.create<dai::node::ColorCamera>();
    auto nn = p.create<dai::node::NeuralNetwork>();
    auto rgbOut = p.create<dai::node::XLinkOut>();
    auto cast = p.create<dai::node::Cast>();
    auto castXout = p.create<dai::node::XLinkOut>();

    camRgb->setPreviewSize(SHAPE, SHAPE);
    camRgb->setInterleaved(false);

    nn->setBlobPath(BLOB_PATH);

    rgbOut->setStreamName("rgb");
    castXout->setStreamName("cast");

    cast->setOutputFrameType(dai::ImgFrame::Type::BGR888p);

    // Linking
    camRgb->preview.link(nn->input);
    camRgb->preview.link(rgbOut->input);
    nn->out.link(cast->input);
    cast->output.link(castXout->input);

    dai::Device device(p);
    auto qCam = device.getOutputQueue("rgb", 4, false);
    auto qCast = device.getOutputQueue("cast", 4, false);

    while(true) {
        auto inCast = qCast->get<dai::ImgFrame>();
        auto inRgb = qCam->get<dai::ImgFrame>();

        if(inCast && inRgb) {
            cv::imshow("Blur", inCast->getCvFrame());
            cv::imshow("Original", inRgb->getCvFrame());
        }

        if(cv::waitKey(1) == 'q') {
            break;
        }
    }

    return 0;
}