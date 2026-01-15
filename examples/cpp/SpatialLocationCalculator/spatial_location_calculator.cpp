#include <atomic>
#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>

#include "depthai/depthai.hpp"

int main() {
    try {
        // Create pipeline
        dai::Pipeline pipeline;

        // Define sources and outputs
        auto monoLeft = pipeline.create<dai::node::Camera>();
        monoLeft->build(dai::CameraBoardSocket::CAM_B);

        auto monoRight = pipeline.create<dai::node::Camera>();
        monoRight->build(dai::CameraBoardSocket::CAM_C);

        auto stereo = pipeline.create<dai::node::StereoDepth>();
        auto spatialLocationCalculator = pipeline.create<dai::node::SpatialLocationCalculator>();

        // Configure stereo
        stereo->setRectification(true);
        stereo->setExtendedDisparity(true);

        // Initial ROI configuration
        dai::Point2f topLeft(0.4f, 0.4f);
        dai::Point2f bottomRight(0.6f, 0.6f);
        float stepSize = 0.05f;

        // Configure spatial location calculator
        dai::SpatialLocationCalculatorConfigData config;
        config.depthThresholds.lowerThreshold = 10;
        config.depthThresholds.upperThreshold = 10000;
        auto calculationAlgorithm = dai::SpatialLocationCalculatorAlgorithm::MEDIAN;
        config.roi = dai::Rect(topLeft, bottomRight);

        spatialLocationCalculator->inputConfig.setWaitForMessage(false);
        spatialLocationCalculator->initialConfig->addROI(config);

        // Create output queues
        auto xoutSpatialQueue = spatialLocationCalculator->out.createOutputQueue();
        auto outputDepthQueue = spatialLocationCalculator->passthroughDepth.createOutputQueue();
        auto inputConfigQueue = spatialLocationCalculator->inputConfig.createInputQueue();

        // Linking
        monoLeft->requestOutput(std::make_pair(640, 400))->link(stereo->left);
        monoRight->requestOutput(std::make_pair(640, 400))->link(stereo->right);
        stereo->depth.link(spatialLocationCalculator->inputDepth);

        // Start pipeline
        pipeline.start();

        cv::Scalar color(255, 255, 255);

        while(pipeline.isRunning()) {
            auto spatialData = xoutSpatialQueue->get<dai::SpatialLocationCalculatorData>();
            std::cout << "Use WASD keys to move ROI!" << std::endl;

            auto outputDepthImage = outputDepthQueue->get<dai::ImgFrame>();
            cv::Mat frameDepth = outputDepthImage->getCvFrame();

            // Calculate median depth
            std::vector<float> depthValues;
            for(int i = 0; i < frameDepth.rows; i++) {
                for(int j = 0; j < frameDepth.cols; j++) {
                    uint16_t val = frameDepth.at<uint16_t>(i, j);
                    if(val > 0) depthValues.push_back(val);
                }
            }
            std::sort(depthValues.begin(), depthValues.end());
            float medianDepth = depthValues[depthValues.size() / 2];
            std::cout << "Median depth value: " << medianDepth << std::endl;

            // Process depth frame for visualization
            cv::Mat depthFrameColor;
            cv::normalize(frameDepth, depthFrameColor, 255, 0, cv::NORM_INF, CV_8UC1);
            cv::equalizeHist(depthFrameColor, depthFrameColor);
            cv::applyColorMap(depthFrameColor, depthFrameColor, cv::COLORMAP_HOT);

            // Draw spatial data
            for(const auto& depthData : spatialData->spatialLocations) {
                auto roi = depthData.config.roi;
                roi = roi.denormalize(depthFrameColor.cols, depthFrameColor.rows);
                int xmin = static_cast<int>(roi.topLeft().x);
                int ymin = static_cast<int>(roi.topLeft().y);
                int xmax = static_cast<int>(roi.bottomRight().x);
                int ymax = static_cast<int>(roi.bottomRight().y);

                float depthMin = depthData.depthMin;
                float depthMax = depthData.depthMax;

                cv::rectangle(depthFrameColor, cv::Point(xmin, ymin), cv::Point(xmax, ymax), color, 1);
                cv::putText(depthFrameColor,
                            "X: " + std::to_string(static_cast<int>(depthData.spatialCoordinates.x)) + " mm",
                            cv::Point(xmin + 10, ymin + 20),
                            cv::FONT_HERSHEY_TRIPLEX,
                            0.5,
                            color);
                cv::putText(depthFrameColor,
                            "Y: " + std::to_string(static_cast<int>(depthData.spatialCoordinates.y)) + " mm",
                            cv::Point(xmin + 10, ymin + 35),
                            cv::FONT_HERSHEY_TRIPLEX,
                            0.5,
                            color);
                cv::putText(depthFrameColor,
                            "Z: " + std::to_string(static_cast<int>(depthData.spatialCoordinates.z)) + " mm",
                            cv::Point(xmin + 10, ymin + 50),
                            cv::FONT_HERSHEY_TRIPLEX,
                            0.5,
                            color);
            }

            cv::imshow("depth", depthFrameColor);

            int key = cv::waitKey(1);
            bool newConfig = false;

            if(key == 'q') {
                break;
            } else if(key == 'w') {
                if(topLeft.y - stepSize >= 0) {
                    topLeft.y -= stepSize;
                    bottomRight.y -= stepSize;
                    newConfig = true;
                }
            } else if(key == 'a') {
                if(topLeft.x - stepSize >= 0) {
                    topLeft.x -= stepSize;
                    bottomRight.x -= stepSize;
                    newConfig = true;
                }
            } else if(key == 's') {
                if(bottomRight.y + stepSize <= 1) {
                    topLeft.y += stepSize;
                    bottomRight.y += stepSize;
                    newConfig = true;
                }
            } else if(key == 'd') {
                if(bottomRight.x + stepSize <= 1) {
                    topLeft.x += stepSize;
                    bottomRight.x += stepSize;
                    newConfig = true;
                }
            } else if(key == '1') {
                calculationAlgorithm = dai::SpatialLocationCalculatorAlgorithm::MEAN;
                std::cout << "Switching calculation algorithm to MEAN!" << std::endl;
                newConfig = true;
            } else if(key == '2') {
                calculationAlgorithm = dai::SpatialLocationCalculatorAlgorithm::MIN;
                std::cout << "Switching calculation algorithm to MIN!" << std::endl;
                newConfig = true;
            } else if(key == '3') {
                calculationAlgorithm = dai::SpatialLocationCalculatorAlgorithm::MAX;
                std::cout << "Switching calculation algorithm to MAX!" << std::endl;
                newConfig = true;
            } else if(key == '4') {
                calculationAlgorithm = dai::SpatialLocationCalculatorAlgorithm::MODE;
                std::cout << "Switching calculation algorithm to MODE!" << std::endl;
                newConfig = true;
            } else if(key == '5') {
                calculationAlgorithm = dai::SpatialLocationCalculatorAlgorithm::MEDIAN;
                std::cout << "Switching calculation algorithm to MEDIAN!" << std::endl;
                newConfig = true;
            }

            if(newConfig) {
                config.roi = dai::Rect(topLeft, bottomRight);
                config.calculationAlgorithm = calculationAlgorithm;
                std::shared_ptr<dai::SpatialLocationCalculatorConfig> cfg = std::make_shared<dai::SpatialLocationCalculatorConfig>();
                cfg->addROI(config);
                inputConfigQueue->send(cfg);
            }
        }

        // Cleanup
        pipeline.stop();
        pipeline.wait();

    } catch(const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}