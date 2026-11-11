/**
 * Rectify a calibrated perspective stereo pair (matching SDK/firmware required).
 * Try --alpha 0 for cropping, --alpha 1 for more source pixels, or a negative value
 * for extra cropping. Alpha is configured before starting the pipeline.
 * Both platforms use grayscale. On RVC4, try --backend GPU or --backend HW for EVA.
 */
#include <argparse/argparse.hpp>
#include <cmath>
#include <depthai/depthai.hpp>
#include <iostream>
#include <map>
#include <opencv2/opencv.hpp>

cv::Mat stereoView(const std::shared_ptr<dai::ImgFrame>& left, const std::shared_ptr<dai::ImgFrame>& right, bool guides = false) {
    auto leftImage = left->getCvFrame();
    auto rightImage = right->getCvFrame();
    if(leftImage.channels() == 1) cv::cvtColor(leftImage, leftImage, cv::COLOR_GRAY2BGR);
    if(rightImage.channels() == 1) cv::cvtColor(rightImage, rightImage, cv::COLOR_GRAY2BGR);
    cv::Mat view;
    cv::hconcat(leftImage, rightImage, view);
    if(guides) {
        for(int y = 40; y < view.rows; y += 40) cv::line(view, {0, y}, {view.cols - 1, y}, {0, 255, 0}, 1);
    }
    return view;
}

int main(int argc, char** argv) {
    argparse::ArgumentParser program("rectification_alpha");
    program.add_argument("--alpha").default_value(0.0f).scan<'g', float>().help("0: crop, 1: retain more pixels; negative: crop further");
    program.add_argument("--backend").default_value(std::string("AUTO")).choices("AUTO", "CPU", "GPU", "HW").help("Rectification backend");
    try {
        program.parse_args(argc, argv);
        const auto alpha = program.get<float>("--alpha");
        const auto backend = program.get<std::string>("--backend");
        if(!std::isfinite(alpha)) throw std::invalid_argument("--alpha must be finite");

        dai::Pipeline pipeline;
        auto device = pipeline.getDefaultDevice();
        const bool isRvc2 = device->getPlatform() == dai::Platform::RVC2;
        if(isRvc2 && backend != "AUTO" && backend != "HW") throw std::runtime_error("RVC2 supports AUTO and HW backends only");
        const auto pairs = device->getAvailableStereoPairs();
        if(pairs.empty()) throw std::runtime_error("This example requires a calibrated stereo camera pair");
        const auto pair = pairs.front();
        auto left = pipeline.create<dai::node::Camera>()->build(pair.left);
        auto right = pipeline.create<dai::node::Camera>()->build(pair.right);
        const auto frameType = dai::ImgFrame::Type::GRAY8;

        auto rectification = pipeline.create<dai::node::Rectification>();
        using Backend = dai::node::Rectification::Backend;
        const std::map<std::string, Backend> backends = {{"AUTO", Backend::AUTO}, {"CPU", Backend::CPU}, {"GPU", Backend::GPU}, {"HW", Backend::HW}};
        rectification->setBackend(backends.at(backend));
        rectification->setAlphaScaling(alpha);
        // Undistortion, rectification and alpha scaling are combined into one map per camera.
        left->requestOutput({640, 400}, frameType, dai::ImgResizeMode::CROP, 15.0f, false)->link(rectification->input1);
        right->requestOutput({640, 400}, frameType, dai::ImgResizeMode::CROP, 15.0f, false)->link(rectification->input2);

        // Keep display queues small and nonblocking so previews cannot exhaust device frame pools.
        auto originalLeft = rectification->passthrough1.createOutputQueue(1, false);
        auto originalRight = rectification->passthrough2.createOutputQueue(1, false);
        auto rectifiedLeft = rectification->output1.createOutputQueue(1, false);
        auto rectifiedRight = rectification->output2.createOutputQueue(1, false);
        const auto title = "Rectified: alpha=" + std::to_string(alpha) + ", backend=" + backend;
        std::cout << "Stereo pair: " << dai::toString(pair.left) << ", " << dai::toString(pair.right) << ". Press q to quit.\n";
        pipeline.start();
        while(pipeline.isRunning()) {
            auto leftOriginal = originalLeft->get<dai::ImgFrame>();
            auto rightOriginal = originalRight->get<dai::ImgFrame>();
            auto leftRectified = rectifiedLeft->get<dai::ImgFrame>();
            auto rightRectified = rectifiedRight->get<dai::ImgFrame>();
            cv::imshow("Original stereo", stereoView(leftOriginal, rightOriginal));
            cv::imshow(title, stereoView(leftRectified, rightRectified, true));
            if(cv::waitKey(1) == 'q') break;
        }
        pipeline.stop();
        pipeline.wait();
        cv::destroyAllWindows();
    } catch(const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
    return 0;
}
