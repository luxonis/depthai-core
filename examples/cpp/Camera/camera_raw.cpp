#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <vector>

#include "depthai/depthai.hpp"

cv::Mat unpackRaw10(const std::vector<uint8_t>& rawData, int width, int height, int stride = -1) {
    if(stride == -1) {
        stride = width * 10 / 8;
    }
    int expectedSize = stride * height;

    if(rawData.size() < expectedSize) {
        throw std::runtime_error("Data too small: " + std::to_string(rawData.size()) + " bytes, expected " + std::to_string(expectedSize));
    }

    // Create output matrix
    cv::Mat result(height, width, CV_16UC1);

    // Process image row by row to handle stride correctly
    for(int row = 0; row < height; row++) {
        // Get row data using stride
        const uint8_t* rowStart = rawData.data() + row * stride;

        // Calculate how many complete 5-byte groups we need for width pixels
        int numGroups = (width + 3) / 4;  // Ceiling division
        int rowBytes = numGroups * 5;

        // Ensure we don't go beyond available data
        if(rowBytes > stride) break;

        // Process each 5-byte group
        for(int i = 0; i < numGroups; i++) {
            const uint8_t* group = rowStart + i * 5;
            uint16_t pixels[4];

            // Extract 8 most significant bits
            pixels[0] = (group[0] << 2);
            pixels[1] = (group[1] << 2);
            pixels[2] = (group[2] << 2);
            pixels[3] = (group[3] << 2);

            // Extract least significant 2 bits from 5th byte
            pixels[0] |= (group[4] & 0b00000011);
            pixels[1] |= ((group[4] & 0b00001100) >> 2);
            pixels[2] |= ((group[4] & 0b00110000) >> 4);
            pixels[3] |= ((group[4] & 0b11000000) >> 6);

            // Copy pixels to result
            for(int j = 0; j < 4 && (i * 4 + j) < width; j++) {
                result.at<uint16_t>(row, i * 4 + j) = pixels[j] * 64;  // Scale from 10-bit to 16-bit
            }
        }
    }

    return result;
}

int main() {
    // Create device
    std::shared_ptr<dai::Device> device = std::make_shared<dai::Device>();

    // Create pipeline
    dai::Pipeline pipeline(device);

    // Create nodes
    auto cam = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);
    auto rawQueue = cam->raw.createOutputQueue();
    auto videoQueue = cam->requestFullResolutionOutput()->createOutputQueue();

    // Start pipeline
    pipeline.start();

    while(true) {
        auto videoIn = videoQueue->tryGet<dai::ImgFrame>();
        auto rawFrame = rawQueue->tryGet<dai::ImgFrame>();

        if(rawFrame != nullptr) {
            auto dataRaw = rawFrame->getData();
            std::vector<uint8_t> dataRawVec(dataRaw.begin(), dataRaw.end());
            try {
                cv::Mat parsedImage = unpackRaw10(dataRawVec, rawFrame->getWidth(), rawFrame->getHeight(), rawFrame->getStride());
                cv::imshow("raw", parsedImage);
            } catch(const std::exception& e) {
                std::cerr << "Error processing raw frame: " << e.what() << std::endl;
            }
        }

        if(videoIn != nullptr) {
            cv::imshow("video", videoIn->getCvFrame());
        }

        if(cv::waitKey(1) == 'q') {
            break;
        }
    }

    return 0;
}