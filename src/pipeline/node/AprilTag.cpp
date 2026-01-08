#include "depthai/pipeline/node/AprilTag.hpp"

#include <math.h>

#include <stdexcept>

#include "pipeline/ThreadedNodeImpl.hpp"
#include "pipeline/datatype/AprilTagConfig.hpp"
#include "properties/AprilTagProperties.hpp"

#ifdef DEPTHAI_HAS_APRIL_TAG

    #ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
        #include <opencv2/imgproc.hpp>
    #endif

    #include "depthai/pipeline/datatype/AprilTags.hpp"
    #include "pipeline/datatype/ImgFrame.hpp"

extern "C" {
    #include "apriltag.h"
    #include "tag16h5.h"
    #include "tag25h9.h"
    #include "tag36h10.h"
    #include "tag36h11.h"
    #include "tagCircle21h7.h"
    #include "tagStandard41h12.h"
}

#endif

namespace dai {
namespace node {

AprilTag::AprilTag(std::unique_ptr<Properties> props)
    : DeviceNodeCRTP<DeviceNode, AprilTag, AprilTagProperties>(std::move(props)),
      initialConfig(std::make_shared<decltype(properties.initialConfig)>(properties.initialConfig)) {}

AprilTag::Properties& AprilTag::getProperties() {
    properties.initialConfig = *initialConfig;
    return properties;
}

// Node properties configuration
void AprilTag::setWaitForConfigInput(bool wait) {
    properties.inputConfigSync = wait;
}

bool AprilTag::getWaitForConfigInput() const {
    return properties.inputConfigSync;
}

void AprilTag::setNumThreads(int numThreads) {
    properties.numThreads = numThreads;
}

int AprilTag::getNumThreads() const {
    return properties.numThreads;
}

void AprilTag::setRunOnHost(bool runOnHost) {
    runOnHostVar = runOnHost;
}

bool AprilTag::runOnHost() const {
    return runOnHostVar;
}

void AprilTag::buildInternal() {
    if(device) {
        auto platform = device->getPlatform();
        runOnHostVar = platform == Platform::RVC2 || platform == Platform::RVC3;
    } else {
        // No device, default to host
        runOnHostVar = true;
    }
    pimpl->logger->info("AprilTag node running on host: {}", runOnHostVar);
}

#ifdef DEPTHAI_HAS_APRIL_TAG

apriltag_family_t* getAprilTagFamily(dai::AprilTagConfig::Family family) {
    apriltag_family_t* tf = nullptr;
    switch(family) {
        case dai::AprilTagConfig::Family::TAG_36H11:
            tf = tag36h11_create();
            break;
        case dai::AprilTagConfig::Family::TAG_36H10:
            tf = tag36h10_create();
            break;
        case dai::AprilTagConfig::Family::TAG_25H9:
            tf = tag25h9_create();
            break;
        case dai::AprilTagConfig::Family::TAG_16H5:
            tf = tag16h5_create();
            break;
        case dai::AprilTagConfig::Family::TAG_CIR21H7:
            tf = tagCircle21h7_create();
            break;
        case dai::AprilTagConfig::Family::TAG_STAND41H12:
            tf = tagStandard41h12_create();
            break;
        default:
            throw std::runtime_error("Unsupported AprilTag family");
    }
    return tf;
}

void destroyAprilTagFamily(apriltag_family_t* tf, dai::AprilTagConfig::Family family) {
    if(tf == nullptr) {
        return;
    }

    switch(family) {
        case dai::AprilTagConfig::Family::TAG_36H11:
            tag36h11_destroy(tf);
            break;
        case dai::AprilTagConfig::Family::TAG_36H10:
            tag36h10_destroy(tf);
            break;
        case dai::AprilTagConfig::Family::TAG_25H9:
            tag25h9_destroy(tf);
            break;
        case dai::AprilTagConfig::Family::TAG_16H5:
            tag16h5_destroy(tf);
            break;
        case dai::AprilTagConfig::Family::TAG_CIR21H7:
            tagCircle21h7_destroy(tf);
            break;
        case dai::AprilTagConfig::Family::TAG_STAND41H12:
            tagStandard41h12_destroy(tf);
            break;
        default:
            throw std::runtime_error("Unsupported AprilTag family");
    }
}

void handleErrors(int e) {
    // Hamming distance error
    if(e == ENOMEM) {
        throw std::runtime_error("AprilTag node: Unable to add family to detector due to insufficient memory to allocate the tag-family decoder.");
    }

    // Memory allocation error
    if(e == EINVAL) {
        throw std::runtime_error("AprilTag node: memory error");
    }
}

void setDetectorConfig(apriltag_detector_t* td, apriltag_family_t* tf, AprilTagConfig::Family& family, const dai::AprilTagConfig& config) {
    // Remove old detector family
    apriltag_detector_clear_families(td);

    // Destroy old detector family
    destroyAprilTagFamily(tf, family);

    // Set new detector family
    apriltag_detector_add_family(td, getAprilTagFamily(config.family));
    family = config.family;

    // Set detector config
    td->quad_decimate = static_cast<float>(config.quadDecimate);
    td->quad_sigma = config.quadSigma;
    td->refine_edges = config.refineEdges;
    td->decode_sharpening = static_cast<double>(config.decodeSharpening);

    // Set detector thresholds
    td->qtp.min_cluster_pixels = config.quadThresholds.minClusterPixels;
    td->qtp.critical_rad = static_cast<float>((static_cast<double>(config.quadThresholds.criticalDegree) * (M_PI / 180.0)));  // Convert degrees to radians
    td->qtp.cos_critical_rad = cos(td->qtp.critical_rad);
    td->qtp.max_line_fit_mse = config.quadThresholds.maxLineFitMse;
    td->qtp.deglitch = static_cast<int>(config.quadThresholds.deglitch);

    // We don't want to debug
    td->debug = false;
}

void setDetectorProperties(apriltag_detector_t* td, const dai::AprilTagProperties& properties) {
    td->nthreads = properties.numThreads;
}

void AprilTag::run() {
    auto& logger = pimpl->logger;
    // Retrieve properties and initial config
    const dai::AprilTagProperties& properties = getProperties();
    dai::AprilTagConfig config = properties.initialConfig;

    // Prepare other variables
    std::shared_ptr<ImgFrame> inFrame = nullptr;
    std::shared_ptr<AprilTagConfig> inConfig = nullptr;
    #ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    std::unique_ptr<cv::Mat> cvimgPtr = nullptr;
    #endif

    // Setup april tag detector
    apriltag_family_t* tf = nullptr;
    AprilTagConfig::Family tfamily = config.family;
    std::unique_ptr<apriltag_detector_t, void (*)(apriltag_detector_t*)> td(apriltag_detector_create(), apriltag_detector_destroy);

    // Set detector properties
    setDetectorProperties(td.get(), properties);

    // Set detector config
    setDetectorConfig(td.get(), tf, tfamily, config);

    // Handle possible errors during configuration
    handleErrors(errno);

    while(mainLoop()) {
        // Preallocate data on stack for AprilTag detection
        int32_t width = 0;
        int32_t height = 0;
        int32_t stride = 0;
        uint8_t* imgbuf = nullptr;
        ImgFrame::Type frameType = ImgFrame::Type::NONE;

        {
            auto blockEvent = this->inputBlockEvent();

            // Retrieve config from user if available
            if(properties.inputConfigSync) {
                inConfig = inputConfig.get<AprilTagConfig>();
            } else {
                inConfig = inputConfig.tryGet<AprilTagConfig>();
            }

            // Set config if there is one and handle possible errors
            if(inConfig != nullptr) {
                setDetectorConfig(td.get(), tf, tfamily, *inConfig);
                handleErrors(errno);
            }

            // Get latest frame
            inFrame = inputImage.get<ImgFrame>();

            // Prepare data for AprilTag detection based on input frame type
            frameType = inFrame->getType();
        }

        if(frameType == ImgFrame::Type::GRAY8 || frameType == ImgFrame::Type::NV12) {
            width = static_cast<int32_t>(inFrame->getWidth());
            height = static_cast<int32_t>(inFrame->getHeight());
            stride = static_cast<int32_t>(inFrame->getStride());
            imgbuf = inFrame->data->getData().data() + inFrame->fb.p1Offset;
        } else {
    #ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
            cvimgPtr = std::make_unique<cv::Mat>();
            cv::cvtColor(inFrame->getCvFrame(), *cvimgPtr, cv::COLOR_BGR2GRAY);
            width = cvimgPtr->cols;
            height = cvimgPtr->rows;
            stride = cvimgPtr->cols;
            imgbuf = cvimgPtr->data;
    #else
            throw std::runtime_error("AprilTag node: Unsupported frame type without opencv support, only GRAY8 and NV12 supported");
    #endif
        }

        // Create AprilTag image
        image_u8_t aprilImg{width, height, stride, imgbuf};

        // Detect AprilTags
        auto now = std::chrono::system_clock::now();
        std::unique_ptr<zarray_t, void (*)(zarray_t*)> detections(apriltag_detector_detect(td.get(), &aprilImg), apriltag_detections_destroy);
        auto end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsedSeconds = end - now;
        logger->trace("April detections took {} ms", elapsedSeconds.count() / 1000.0);

        std::shared_ptr<dai::AprilTags> aprilTags = std::make_shared<dai::AprilTags>();

        if(detections != nullptr) {
            int numDetections = zarray_size(detections.get());
            aprilTags->aprilTags.reserve(numDetections);
            for(int i = 0; i < numDetections; i++) {
                apriltag_detection_t* det = nullptr;
                zarray_get(detections.get(), i, &det);
                if(det == nullptr) {
                    continue;
                }
                dai::AprilTag daiDet;
                daiDet.id = det->id;
                daiDet.hamming = det->hamming;
                daiDet.decisionMargin = det->decision_margin;
                dai::Point2f center;
                center.x = static_cast<float>(det->c[0]);
                center.y = static_cast<float>(det->c[1]);

                dai::Point2f topLeft;
                dai::Point2f topRight;
                dai::Point2f bottomRight;
                dai::Point2f bottomLeft;

                topLeft.x = static_cast<float>(det->p[3][0]);
                topLeft.y = static_cast<float>(det->p[3][1]);
                topRight.x = static_cast<float>(det->p[2][0]);
                topRight.y = static_cast<float>(det->p[2][1]);
                bottomRight.x = static_cast<float>(det->p[1][0]);
                bottomRight.y = static_cast<float>(det->p[1][1]);
                bottomLeft.x = static_cast<float>(det->p[0][0]);
                bottomLeft.y = static_cast<float>(det->p[0][1]);

                daiDet.topLeft = topLeft;
                daiDet.topRight = topRight;
                daiDet.bottomRight = bottomRight;
                daiDet.bottomLeft = bottomLeft;

                aprilTags->aprilTags.push_back(daiDet);
            }
        }

        // Inherit sequence number and timestamp from input image
        aprilTags->setSequenceNum(inFrame->getSequenceNum());
        aprilTags->setTimestamp(inFrame->getTimestamp());
        aprilTags->setTimestampDevice(inFrame->getTimestampDevice());
        aprilTags->setTimestampSystem(inFrame->getTimestampSystem());

        {
            auto blockEvent = this->outputBlockEvent();

            // Send detections and pass through input frame
            out.send(aprilTags);
            passthroughInputImage.send(inFrame);
        }

        // Logging
        logger->trace("Detected {} april tags", zarray_size(detections.get()));
    }

    // Destroy AprilTag family
    destroyAprilTagFamily(tf, tfamily);
}

#else

void AprilTag::run() {
    throw std::runtime_error("AprilTag node is not supported on Windows");
}

#endif

}  // namespace node
}  // namespace dai
