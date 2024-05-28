#include <depthai/depthai.hpp>
#include <filesystem>
#include <opencv2/opencv.hpp>

constexpr int SHAPE = 720;

int main() {
    dai::Pipeline p;

    auto camRgb = p.create<dai::node::ColorCamera>();
    auto nn = p.create<dai::node::NeuralNetwork>();
    auto script = p.create<dai::node::Script>();
    auto rgbXout = p.create<dai::node::XLinkOut>();
    auto cast = p.create<dai::node::Cast>();
    auto castXout = p.create<dai::node::XLinkOut>();

    camRgb->setVideoSize(SHAPE, SHAPE);
    camRgb->setPreviewSize(SHAPE, SHAPE);
    camRgb->setInterleaved(false);

    nn->setBlobPath(BLOB_PATH);

    script->setScript(R"(
        old = node.io['in'].get()
        while True:
            frame = node.io['in'].get()
            node.io['img1'].send(old)
            node.io['img2'].send(frame)
            old = frame
    )");

    rgbXout->setStreamName("rgb");
    castXout->setStreamName("cast");
    cast->setOutputFrameType(dai::RawImgFrame::Type::GRAY8);

    // Linking
    camRgb->preview.link(script->inputs["in"]);
    script->outputs["img1"].link(nn->inputs["img1"]);
    script->outputs["img2"].link(nn->inputs["img2"]);
    camRgb->video.link(rgbXout->input);
    nn->out.link(cast->input);
    cast->output.link(castXout->input);

    // Pipeline is defined, now we can connect to the device
    dai::Device device(p);
    auto qCam = device.getOutputQueue("rgb", 4, false);
    auto qCast = device.getOutputQueue("cast", 4, false);

    while(true) {
        auto colorFrame = qCam->get<dai::ImgFrame>();
        if(colorFrame) {
            cv::imshow("Color", colorFrame->getCvFrame());
        }

        auto inCast = qCast->get<dai::ImgFrame>();
        if(inCast) {
            cv::imshow("Diff", inCast->getCvFrame());
        }

        if(cv::waitKey(1) == 'q') {
            break;
        }
    }

    return 0;
}