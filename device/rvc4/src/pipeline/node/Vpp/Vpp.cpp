#include "Vpp.hpp"

#include "utility/ImgFrameView.hpp"

// std
#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <thread>

// OpenCV
#include <opencv2/opencv.hpp>

// project

#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/datatype/VppConfig.hpp"
#include "pipeline/NodeTiming.hpp"

namespace dai {
namespace gate {

cv::Mat Vpp::convertUintMatToFloat(const cv::Mat& matUint) {
    // Validate input
    if(matUint.empty()) {
        throw std::invalid_argument("convertMatToFloat: Input mat image is empty.");
    }

    if(matUint.depth() != CV_8U && matUint.depth() != CV_16U) {
        throw std::invalid_argument("convertMatToFloat: Unsupported input type. Expected CV_8U or CV_16U.");
    }

    // Convert to float
    cv::Mat matFloat;
    matUint.convertTo(matFloat, CV_32F, 1. / 16.);

    return matFloat;
}

void Vpp::updateDepthToDisparityLut(double scale) {
    if(!depthToDisparityLut.empty() && scale == cachedDepthToDisparityScale) return;

    depthToDisparityLut.resize(65536);
    depthToDisparityLut[0] = 0.0f;
    for(size_t value = 1; value < depthToDisparityLut.size(); ++value) {
        // Truncate to unsigned Q4 before converting to pixel disparity, matching the replay reference.
        const auto q4 = static_cast<uint16_t>(std::min(scale / value, 65535.0));
        depthToDisparityLut[value] = q4 / 16.0f;
    }
    cachedDepthToDisparityScale = scale;
}

cv::Mat Vpp::buildDisparityPrior(const cv::Mat& depth, float focalLength, float baselineMm) {
    if(depth.empty()) {
        throw std::invalid_argument("VPP depth must be nonempty");
    }
    if(!std::isfinite(focalLength) || focalLength <= 0.0f) {
        throw std::invalid_argument("VPP depth prior requires a finite positive focal length");
    }
    const double numerator = static_cast<double>(focalLength) * baselineMm * 16.0;
    updateDepthToDisparityLut(numerator);
    cv::Mat disparity(depth.size(), CV_32FC1);
    for(int y = 0; y < depth.rows; ++y) {
        const auto* depthRow = depth.ptr<uint16_t>(y);
        auto* disparityRow = disparity.ptr<float>(y);
        for(int x = 0; x < depth.cols; ++x) {
            disparityRow[x] = depthToDisparityLut[depthRow[x]];
        }
    }
    return disparity;
}

float Vpp::resolveBaselineMm(const std::shared_ptr<dai::ImgFrame>& leftIn, const std::shared_ptr<dai::ImgFrame>& rightIn) {
    if(baselineMm > 0.0f) return baselineMm;

    // Use measured translation from frame extrinsics, not the CAD design value.
    const auto translation = leftIn->getTransformation().getTranslationVectorTo(rightIn->getTransformation(), false, LengthUnit::MILLIMETER);
    const float fromFrames = std::hypot(translation[0], translation[1], translation[2]);
    if(!std::isfinite(fromFrames) || fromFrames <= 0.0f) {
        throw std::runtime_error("VPP depth input requires a finite positive stereo baseline from frame extrinsics");
    }
    baselineMm = fromFrames;
    logger->info("VPP node: stereo baseline {:.3f} mm from frame extrinsics", baselineMm);
    return baselineMm;
}

void Vpp::sendData(const std::shared_ptr<dai::ImgFrame>& leftOutFrame, const std::shared_ptr<dai::ImgFrame>& rightOutFrame) {
    {
        auto blockEvent = this->outputBlockEvent();
        // Send output frames
        sendLeft(leftOutFrame);
        sendRight(rightOutFrame);
    }
}

std::shared_ptr<MessageGroup> Vpp::getMessageGroup() {
    return syncedInputs.get<MessageGroup>();
}

void Vpp::copyFrameToGray(cv::Mat& dst, const cv::Mat& src) {
    if(src.empty()) {
        throw std::runtime_error("Vpp::copyFrameToGray: Source frame is empty.");
    }

    switch(src.channels()) {
        case 1:
            src.copyTo(dst);
            break;
        case 3:
            // Convert BGR → Grayscale
            cv::cvtColor(src, dst, cv::COLOR_BGR2GRAY);
            break;
        case 4:
            // Convert BGRA → Grayscale
            cv::cvtColor(src, dst, cv::COLOR_BGRA2GRAY);
            break;
        default:
            throw std::runtime_error("Vpp::copyFrameToGray: Unsupported number of channels (" + std::to_string(src.channels()) + ").");
    }
}

cv::Mat Vpp::prepareGrayOutputFrame(std::shared_ptr<dai::ImgFrame>& outputFrame, const std::shared_ptr<dai::ImgFrame>& inputFrame) {
    if(!inputFrame) {
        throw std::invalid_argument("Vpp::prepareGrayOutputFrame: Input frame is null.");
    }

    constexpr uint32_t hwHeightAlignment = 32;
    const auto alignedHeight = (inputFrame->getHeight() + hwHeightAlignment - 1) / hwHeightAlignment * hwHeightAlignment;
    const auto outputSize = static_cast<size_t>(inputFrame->getWidth()) * alignedHeight;
    outputFrame = std::make_shared<ImgFrame>(outputSize);
    outputFrame->setMetadata(inputFrame);
    outputFrame->setType(ImgFrame::Type::GRAY8);
    outputFrame->setSize(inputFrame->getWidth(), inputFrame->getHeight());
    outputFrame->setStride(inputFrame->getWidth());

    return cv::Mat(inputFrame->getHeight(), inputFrame->getWidth(), CV_8UC1, outputFrame->getData().data(), outputFrame->getStride());
}

bool Vpp::loadData(cv::Mat& leftImg,
                   cv::Mat& rightImg,
                   cv::Mat& disparityCv,
                   cv::Mat& confidenceCv,
                   std::shared_ptr<dai::ImgFrame>& leftOutFrame,
                   std::shared_ptr<dai::ImgFrame>& rightOutFrame) {
    std::shared_ptr<MessageGroup> syncedIn;
    std::shared_ptr<dai::ImgFrame> leftIn;
    std::shared_ptr<dai::ImgFrame> rightIn;
    std::shared_ptr<dai::ImgFrame> disparityIn;
    std::shared_ptr<dai::ImgFrame> confidenceIn;
    std::shared_ptr<dai::ImgFrame> depthIn;
    {
        auto blockEvent = this->inputBlockEvent();
        syncedIn = getMessageGroup();
        if(!syncedIn) return false;
        depthIn = syncedIn->get<ImgFrame>("depth");
        leftIn = syncedIn->get<ImgFrame>(leftInputName);
        rightIn = syncedIn->get<ImgFrame>(rightInputName);
        disparityIn = syncedIn->get<ImgFrame>(disparityName);
        confidenceIn = syncedIn->get<ImgFrame>(confidenceName);
    }
    // Convert left image safely
    if(!leftIn || !rightIn || (!disparityIn && !depthIn)) {
        return false;
    }

    if(depthIn && disparityIn) {
        throw std::invalid_argument("VPP expects either depth or disparity, not both");
    }

    leftImg = prepareGrayOutputFrame(leftOutFrame, leftIn);
    rightImg = prepareGrayOutputFrame(rightOutFrame, rightIn);
    copyFrameToGray(leftImg, utility::getCvFrameView(*leftIn));
    copyFrameToGray(rightImg, utility::getCvFrameView(*rightIn));

    // Verify image size consistency
    CV_Assert(leftImg.size() == rightImg.size());
    if(depthIn) {
        if(depthIn->getType() != ImgFrame::Type::RAW16 || depthIn->getWidth() != leftIn->getWidth() || depthIn->getHeight() != leftIn->getHeight()) {
            throw std::invalid_argument("VPP depth must be RAW16 millimetres aligned to the full rectified left image");
        }
        const auto intrinsics = leftIn->getTransformation().getIntrinsicMatrix();
        disparityCv = buildDisparityPrior(depthIn->getFrame(), intrinsics.at(0).at(0), resolveBaselineMm(leftIn, rightIn));
    } else {
        disparityCv = convertUintMatToFloat(utility::getCvFrameView(*disparityIn));
    }
    if(leftImg.cols < disparityCv.cols || leftImg.rows < disparityCv.rows) {
        throw std::runtime_error(
            "Unsupported disparity size: Disparity map is larger than the left image. "
            "Expected disparity <= left image size, but got "
            + std::to_string(disparityCv.cols) + "x" + std::to_string(disparityCv.rows) + " vs " + std::to_string(leftImg.cols) + "x"
            + std::to_string(leftImg.rows));
    }

    if(!confidenceIn) {
        confidenceCv = cv::Mat();
    } else {
        confidenceCv = convertUintMatToFloat(utility::getCvFrameView(*confidenceIn));
        CV_Assert(disparityCv.size() == confidenceCv.size());
    }
    return true;
}

void Vpp::run() {
    logger->info("{} running on {}.", this->getName(), runOnHost() ? "host" : "device");

    // Initialize VppImpl with initialConfig from properties
    const dai::VppProperties& properties = getProperties();
    vppImpl = std::make_unique<VppImpl>(properties.initialConfig);

    // Log the initial configuration values for verification
    logger->info("VPP node: Initialized with initialConfig:");

    while(mainLoop()) {
        auto tAbsoluteBeginning = std::chrono::steady_clock::now();

        auto vppConfig = inputConfig.tryGet<VppConfig>();
        if(vppConfig) {
            logger->info("VPP node: Setting config ...");
            vppImpl->setConfig(*vppConfig);
            maxFPS = vppConfig->maxFPS;
        }

        // Get input frames
        logger->trace("VPP node: loading data...");
        std::shared_ptr<dai::ImgFrame> leftOutFrame;
        std::shared_ptr<dai::ImgFrame> rightOutFrame;
        cv::Mat leftOutFrameCv, rightOutFrameCv, disparityCv, confidenceCv;

        bool loadedCorreclty = loadData(leftOutFrameCv, rightOutFrameCv, disparityCv, confidenceCv, leftOutFrame, rightOutFrame);
        if(!loadedCorreclty) {
            logger->error("VPP node : Data were not loaded correctly — skipping frame.");
            continue;
        }
        auto tGotInput = std::chrono::steady_clock::now();

        logger->trace("VPP node: data loaded; img resolution = {} x {}; disparity resolution = {} x {}",
                      leftOutFrameCv.cols,
                      leftOutFrameCv.rows,
                      disparityCv.cols,
                      disparityCv.rows);

        if(vppImpl->getConfig().blending > 0.f) {
            logger->trace("Vpp node: inserting virtual patterns ...");
            vppImpl->process(leftOutFrameCv, rightOutFrameCv, disparityCv, confidenceCv);
            logger->trace("Vpp node: inserting done ...");
        }

        auto tProcessed = std::chrono::steady_clock::now();
        sendData(leftOutFrame, rightOutFrame);
        logger->trace("Vpp node: data sent ...");
        auto tAbsoluteEnd = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(tAbsoluteEnd - tAbsoluteBeginning);
        dai::gate::logTiming(logger, this->getName(), tAbsoluteBeginning, tGotInput, tProcessed, tAbsoluteEnd);

        // --- FPS limiting ---
        if(maxFPS > 0) {
            int64_t targetMs = 1000 / maxFPS;
            if(elapsed.count() < targetMs) {
                std::this_thread::sleep_for(std::chrono::milliseconds(targetMs - elapsed.count()));
            }
        }
    }
}

}  // namespace gate
}  // namespace dai
