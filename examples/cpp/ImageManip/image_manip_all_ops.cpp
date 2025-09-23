#include <atomic>
#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <opencv2/opencv.hpp>

#include "depthai/depthai.hpp"

int main() {
    try {
        // Create pipeline
        dai::Pipeline pipeline;

        // Create input manipulator
        auto manipInput = pipeline.create<dai::node::ImageManip>();
        manipInput->initialConfig->setFrameType(dai::ImgFrame::Type::BGR888p);
        auto inputQueue = manipInput->inputImage.createInputQueue();

        // Define manipulation operations
        std::vector<std::pair<std::string, std::function<void(dai::ImageManipConfig&)>>> manipOps = {
            // Resize operations
            {"resize_stretch", [](dai::ImageManipConfig& conf) { conf.setOutputSize(256, 200, dai::ImageManipConfig::ResizeMode::STRETCH); }},
            {"resize_letterbox", [](dai::ImageManipConfig& conf) { conf.setOutputSize(256, 200, dai::ImageManipConfig::ResizeMode::LETTERBOX); }},
            {"resize_center_crop", [](dai::ImageManipConfig& conf) { conf.setOutputSize(256, 200, dai::ImageManipConfig::ResizeMode::CENTER_CROP); }},
            // Crop operation
            {"crop", [](dai::ImageManipConfig& conf) { conf.addCrop(50, 50, 150, 200); }},
            // Flip operations
            {"flip_vertical", [](dai::ImageManipConfig& conf) { conf.addFlipVertical(); }},
            {"flip_horizontal", [](dai::ImageManipConfig& conf) { conf.addFlipHorizontal(); }},
            // Scale operation
            {"scale", [](dai::ImageManipConfig& conf) { conf.addScale(0.7f, 0.5f); }},
            // Rotate operations
            {"rotate_90_deg", [](dai::ImageManipConfig& conf) { conf.addRotateDeg(90); }},
            {"rotate_90_deg_center",
             [](dai::ImageManipConfig& conf) {
                 conf.addRotateDeg(90, dai::Point2f(0.2f, 0.3f));
                 conf.setOutputCenter(false);
             }},
            // Transform operations
            {"transform_affine",
             [](dai::ImageManipConfig& conf) {
                 std::array<float, 4> matrix = {1.0f, 0.5f, 0.2f, 1.0f};
                 conf.addTransformAffine(matrix);
             }},
            {"transform_perspective",
             [](dai::ImageManipConfig& conf) {
                 std::array<float, 9> matrix = {
                     1.0f,
                     0.2f,
                     0.0f,  // First row
                     0.1f,
                     1.0f,
                     0.0f,  // Second row
                     0.001f,
                     0.002f,
                     1.0f  // Third row
                 };
                 conf.addTransformPerspective(matrix);
             }},
            // Frame type conversion
            {"frame_type", [](dai::ImageManipConfig& conf) { conf.setFrameType(dai::ImgFrame::Type::RAW8); }}};

        // Create manipulator nodes and queues
        std::map<std::string, std::shared_ptr<dai::MessageQueue>> queues;
        for(const auto& [name, config] : manipOps) {
            std::cout << "Creating manipulator: " << name << std::endl;
            auto manip = pipeline.create<dai::node::ImageManip>();
            config(*manip->initialConfig);
            manipInput->out.link(manip->inputImage);
            queues[name] = manip->out.createOutputQueue(4, false);
        }

        // Load and prepare input image
        cv::Mat inputFrame = cv::imread(LENNA_PATH);  // 512x512
        if(inputFrame.empty()) {
            throw std::runtime_error("Could not read input image");
        }

        // Create and send input frame
        auto imgFrame = std::make_shared<dai::ImgFrame>();
        cv::Mat downscaled;
        cv::pyrDown(inputFrame, downscaled);
        imgFrame->setCvFrame(downscaled, dai::ImgFrame::Type::BGR888i);
        inputQueue->send(imgFrame);

        // Display input image
        cv::imshow("input_image", inputFrame);

        // Start pipeline
        pipeline.start();

        // Process and display results
        for(const auto& [name, queue] : queues) {
            auto inFrame = queue->get<dai::ImgFrame>();
            cv::imshow(name, inFrame->getCvFrame());
        }

        // Wait for key press
        cv::waitKey(0);

        // Cleanup
        pipeline.stop();
        pipeline.wait();

    } catch(const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}