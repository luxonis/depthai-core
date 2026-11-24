#include "NeuralDepth.hpp"

#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/datatype/NNData.hpp"
#include "depthai/pipeline/datatype/NeuralDepthConfig.hpp"
#include "pipeline/NodeTiming.hpp"

namespace {

cv::Mat tensorToMat32F(xt::xarray<float>& x) {
    auto squeezed = xt::squeeze(x);

    if(squeezed.dimension() != 2) {
        throw std::runtime_error("tensorToMat32F: expected a 2-D tensor after squeezing, got " + std::to_string(squeezed.dimension()) + " dimensions");
    }

    int rows = static_cast<int>(squeezed.shape(0));
    int cols = static_cast<int>(squeezed.shape(1));

    cv::Mat mat(rows, cols, CV_32F, squeezed.data());
    return mat;
}

}  // namespace

namespace dai {
namespace gate {

void disparityToDepth(const cv::Mat& disparity, std::shared_ptr<dai::ImgFrame>& depthFrame, float disp2DepthMult) {
    uint16_t* depth = (std::uint16_t*)depthFrame->data->getData().data();
    float* disparityData = (float*)disparity.data;
    for(int i = 0; i < depthFrame->getWidth() * depthFrame->getHeight(); i++) {
        if(disparityData[i] == 0.f) {
            depth[i] = 0;
        } else {
            depth[i] = disp2DepthMult / disparityData[i];
        }
    }
}

void NeuralDepth::run() {
    logger->info("{} running on {}.", this->getName(), runOnHost() ? "host" : "device");

    float disparityToDepthMultiplier = 0.f;
    bool initialized = false;
    bool newConfigReceived = true;
    std::shared_ptr<dai::ImgFrame> alignedFrame;
    std::shared_ptr<dai::ImgFrame> latestRightFrame;
    dai::ImgTransformation leftCalibrationTransformation;
    dai::ImgTransformation rightCalibrationTransformation;
    dai::NeuralDepthConfig config = *initialConfig;

    while(mainLoop()) {
        if(!initialized) {
            std::shared_ptr<dai::ImgFrame> leftFrame;
            std::shared_ptr<dai::ImgFrame> rightFrame;
            {
                auto blockEvent = this->inputBlockEvent();
                leftFrame = leftInternal.get<dai::ImgFrame>();
                rightFrame = rightInternal.get<dai::ImgFrame>();
            }
            alignedFrame = leftFrame;  // use left frame metadata for output frames
            latestRightFrame = rightFrame;

            initialized = true;
            logger->info("NeuralDepth node initialized");
        }

        std::shared_ptr<dai::ImgFrame> leftFrame;
        std::shared_ptr<dai::ImgFrame> rightFrame;
        std::shared_ptr<dai::NNData> nnResult;
        std::shared_ptr<dai::NeuralDepthConfig> inputConfigMessage;
        {
            auto blockEvent = this->inputBlockEvent();
            nnResult = nnDataInput.get<dai::NNData>();
            leftFrame = leftInternal.tryGet<dai::ImgFrame>();
            rightFrame = rightInternal.tryGet<dai::ImgFrame>();
            inputConfigMessage = inputConfig.tryGet<dai::NeuralDepthConfig>();
        }
        auto tAbsoluteBeginning = std::chrono::steady_clock::now();
        auto tGotInput = tAbsoluteBeginning;
        if(leftFrame) {
            alignedFrame = leftFrame;  // update aligned frame if new left frame is available
        }
        if(rightFrame) {
            latestRightFrame = rightFrame;
        }

        if(inputConfigMessage) {
            config = *inputConfigMessage;
            newConfigReceived = true;
        }

        // The rectified input transformations carry the current calibration, so recompute the depth scale whenever they or the depth unit change
        const auto& leftTransformation = alignedFrame->transformation;
        const auto& rightTransformation = latestRightFrame->transformation;
        if(inputConfigMessage || !leftTransformation.isEqualTransformation(leftCalibrationTransformation)
           || !rightTransformation.isEqualTransformation(rightCalibrationTransformation)) {
            auto M1 = leftTransformation.getIntrinsicMatrix();

            auto T = leftTransformation.getTranslationVectorTo(rightTransformation, false, dai::LengthUnit::CENTIMETER);

            auto baselineIdx = std::abs(T[0]) >= std::abs(T[1]) ? 0 : 1;
            auto baseline = std::abs(T[baselineIdx]) / 100.f;  // meters

            auto focalLength = M1[baselineIdx][baselineIdx];

            float customMultiplier = 1000.f;
            if(config.algorithmControl.depthUnit == dai::DepthUnit::CUSTOM) {
                customMultiplier = config.algorithmControl.customDepthUnitMultiplier;
            } else {
                customMultiplier = dai::getLengthUnitMultiplier(config.algorithmControl.depthUnit);
            }
            disparityToDepthMultiplier = (baseline * customMultiplier) * focalLength;
            logger->debug("NeuralDepth baseline: {} m, focal length: {}, disparityToDepthMultiplier: {}", baseline, focalLength, disparityToDepthMultiplier);

            leftCalibrationTransformation = leftTransformation;
            rightCalibrationTransformation = rightTransformation;
        }

        auto disparityTensor = nnResult->getTensor<float>("disparity", true);
        auto confidenceTensor = nnResult->getTensor<float>("confidence", true);
        auto edgeTensor = nnResult->getTensor<float>("edge", true);

        auto cvDisparity = tensorToMat32F(disparityTensor);
        const auto cvConfidence = tensorToMat32F(confidenceTensor);
        const auto cvEdge = tensorToMat32F(edgeTensor);

        float confidenceThreshold = config.postProcessing.confidenceThreshold / 255.0f;
        float edgeThreshold = static_cast<float>(config.postProcessing.edgeThreshold);
        cv::Mat confidenceMask, edgeMask;
        cv::compare(cvConfidence, confidenceThreshold, confidenceMask, cv::CMP_LT);
        cv::compare(cvEdge, edgeThreshold, edgeMask, cv::CMP_GT);

        cvDisparity.setTo(0.0f, confidenceMask);
        cvDisparity.setTo(0.0f, edgeMask);

        // Temporal filtering
        if(config.postProcessing.temporalFilter.enable) {
            // initialize temporal filter
            if(newConfigReceived) {
                size_t frameMemorySize = cvDisparity.total() * cvDisparity.elemSize();
                float alpha = config.postProcessing.temporalFilter.alpha;
                int delta = config.postProcessing.temporalFilter.delta;
                int persistenceMode = static_cast<int>(config.postProcessing.temporalFilter.persistencyMode);
                temporalFilter.Init(frameMemorySize, alpha, delta, persistenceMode);
            }

            // apply temporal filter
            uint8_t* disparityData = cvDisparity.data;
            size_t frameHeight = cvDisparity.rows;
            size_t frameWidth = cvDisparity.cols;
            temporalFilter.process(disparityData, frameHeight, frameWidth);
        }

        std::shared_ptr<dai::ImgFrame> depthFrame = std::make_shared<dai::ImgFrame>();
        int depthWidth = cvDisparity.cols;
        int depthHeight = cvDisparity.rows;
        size_t depthFrameSize = depthWidth * depthHeight * sizeof(uint16_t);
        depthFrame->setData(std::vector<uint8_t>(depthFrameSize));
        depthFrame->setMetadata(*alignedFrame);
        // Get timestamps from NN data
        depthFrame->setBufferMetadataFrom(nnResult);
        depthFrame->setWidth(depthWidth);
        depthFrame->setHeight(depthHeight);
        depthFrame->sourceFb.width = depthWidth;
        depthFrame->sourceFb.height = depthHeight;
        depthFrame->setType(dai::ImgFrame::Type::RAW16);
        depthFrame->fb.stride = depthFrame->fb.width * depthFrame->getBytesPerPixel();

        // Convert disparity to depth: depth = disparityToDepthMultiplier / disparity
        disparityToDepth(cvDisparity, depthFrame, disparityToDepthMultiplier);
        if(config.postProcessing.thresholdFilter.minRange > 0 || config.postProcessing.thresholdFilter.maxRange < 65535) {
            thresholdFilter.process(depthFrame, config.postProcessing.thresholdFilter.minRange, config.postProcessing.thresholdFilter.maxRange);
        }

        std::shared_ptr<dai::ImgFrame> disparityFrame = std::make_shared<dai::ImgFrame>();
        int disparityWidth = cvDisparity.cols;
        int disparityHeight = cvDisparity.rows;
        size_t disparityFrameSize = disparityWidth * disparityHeight * sizeof(uint16_t);
        disparityFrame->setData(std::vector<uint8_t>(disparityFrameSize));
        disparityFrame->setMetadata(*alignedFrame);
        // Get timestamps from NN data
        disparityFrame->setBufferMetadataFrom(nnResult);
        disparityFrame->setWidth(disparityWidth);
        disparityFrame->setHeight(disparityHeight);
        disparityFrame->sourceFb.width = disparityWidth;
        disparityFrame->sourceFb.height = disparityHeight;
        disparityFrame->setType(dai::ImgFrame::Type::RAW16);
        disparityFrame->fb.stride = disparityFrame->fb.width * disparityFrame->getBytesPerPixel();

        // Convert float cvDisparity to uint16_t with 16 subpixel levels, multiply by 16 and round up
        auto disparityU16 = disparityFrame->getFrame();
        cvDisparity.convertTo(disparityU16, CV_16U, 16.0);

        std::shared_ptr<dai::ImgFrame> confidenceFrame = std::make_shared<dai::ImgFrame>();
        int confidenceWidth = cvConfidence.cols;
        int confidenceHeight = cvConfidence.rows;
        size_t confidenceFrameSize = confidenceWidth * confidenceHeight * sizeof(uint8_t);
        confidenceFrame->setData(std::vector<uint8_t>(confidenceFrameSize));
        confidenceFrame->setMetadata(*alignedFrame);
        // Get timestamps from NN data
        confidenceFrame->setBufferMetadataFrom(nnResult);
        confidenceFrame->setWidth(confidenceWidth);
        confidenceFrame->setHeight(confidenceHeight);
        confidenceFrame->sourceFb.width = confidenceWidth;
        confidenceFrame->sourceFb.height = confidenceHeight;
        confidenceFrame->setType(dai::ImgFrame::Type::RAW8);
        confidenceFrame->fb.stride = confidenceFrame->fb.width * confidenceFrame->getBytesPerPixel();

        // Convert float cvConfidence [0.0, 1.0] to uint8_t [0, 255]
        auto confidenceU8 = confidenceFrame->getFrame();
        cvConfidence.convertTo(confidenceU8, CV_8U, 255.0);

        std::shared_ptr<dai::ImgFrame> edgeFrame = std::make_shared<dai::ImgFrame>();
        int edgeWidth = cvEdge.cols;
        int edgeHeight = cvEdge.rows;
        size_t edgeFrameSize = edgeWidth * edgeHeight * sizeof(uint8_t);
        edgeFrame->setData(std::vector<uint8_t>(edgeFrameSize));
        edgeFrame->setMetadata(*alignedFrame);
        // Get timestamps from NN data
        edgeFrame->setBufferMetadataFrom(nnResult);
        edgeFrame->setWidth(edgeWidth);
        edgeFrame->setHeight(edgeHeight);
        edgeFrame->sourceFb.width = edgeWidth;
        edgeFrame->sourceFb.height = edgeHeight;
        edgeFrame->setType(dai::ImgFrame::Type::RAW8);
        edgeFrame->fb.stride = edgeFrame->fb.width * edgeFrame->getBytesPerPixel();

        auto edgeU8 = edgeFrame->getFrame();
        cvEdge.convertTo(edgeU8, CV_8U);
        auto tProcessed = std::chrono::steady_clock::now();
        {
            auto blockEvent = this->outputBlockEvent();
            disparity.send(disparityFrame);
            confidence.send(confidenceFrame);
            edge.send(edgeFrame);
            depth.send(depthFrame);
        }

        newConfigReceived = false;
        auto tAbsoluteEnd = std::chrono::steady_clock::now();
        dai::gate::logTiming(logger, this->getName(), tAbsoluteBeginning, tGotInput, tProcessed, tAbsoluteEnd);

    }  // while running
}

}  // namespace gate
}  // namespace dai
