#include "depthai/pipeline/node/AprilTag.hpp"
#include "depthai/pipeline/datatype/AprilTags.hpp"

#include <thread>
#include "spdlog/fmt/fmt.h"

extern "C" {
#include "apriltag.h"
#include "common/getopt.h"
#include "tag16h5.h"
#include "tag25h9.h"
#include "tag36h11.h"
#include "tagCircle21h7.h"
#include "tagCircle49h12.h"
#include "tagCustom48h12.h"
#include "tagStandard41h12.h"
#include "tagStandard52h13.h"
}

namespace dai {
namespace node {

AprilTag::AprilTag(std::unique_ptr<Properties> props) : DeviceNodeCRTP<DeviceNode, AprilTag, AprilTagProperties>(std::move(props)) {}

AprilTag::Properties& AprilTag::getProperties() {
    properties.initialConfig = initialConfig;
    return properties;
}

// Node properties configuration
void AprilTag::setWaitForConfigInput(bool wait) {
    properties.inputConfigSync = wait;
}

void AprilTag::setRunOnHost(bool runOnHost) {
    runOnHostVar = runOnHost;
}

bool AprilTag::runOnHost() const {
    return runOnHostVar;
}

void AprilTag::run() {
    std::shared_ptr<ImgFrame> inFrame = nullptr;

    // Initialize tag detector with options
    apriltag_family_t* tf = tag36h11_create();
    apriltag_detector_t* td = apriltag_detector_create();

    apriltag_detector_add_family(td, tf);

    // if(errno == ENOMEM) {
    //         printf(
    //             "Unable to add family to detector due to insufficient memory to allocate the tag-family decoder with the default maximum hamming value of 2.
    //             Try
    //    " "choosing an alternative tag family.\n"); exit(-1);
    // }

    td->quad_decimate = 2.0f;
    td->quad_sigma = 0.0f;
    td->nthreads = 2;
    td->debug = 0;
    td->refine_edges = 1;


    // TODOs:
    // - Handle everything that is settable in properties (family, etc)
    // - In the case of a dynamic config setting different family, etc - handle it
    // - Handle different input types (right now GRAY and NV12 work, but not the rest - not everything needs to be handled, but types that don't work should
    // error out)
    // - Better error handling
    // - Expose number of CPU threads as a property
    while(isRunning()) {
        inFrame = inputImage.get<ImgFrame>();
        // TODO: This only works for types that have a grayscale image at the beginning
        uint8_t* src = inFrame->data->getData().data() + inFrame->fb.p1Offset;
        auto width = static_cast<int32_t>(inFrame->getWidth());
        auto height = static_cast<int32_t>(inFrame->getHeight());
        auto stride = static_cast<int32_t>(inFrame->getStride());
        image_u8_t aprilImg = {.width = width, .height = height, .stride = stride, .buf = src};

        auto now = std::chrono::system_clock::now();
        zarray_t* detections = apriltag_detector_detect(td, &aprilImg);
        auto end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsedSeconds = end - now;
        logger->trace("April detections took {} ms", elapsedSeconds.count() / 1000.0);

        std::shared_ptr<dai::AprilTags> aprilTags = std::make_shared<dai::AprilTags>();

        if(detections != nullptr) {
            int numDetections = zarray_size(detections);
            aprilTags->aprilTags.reserve(numDetections);
            for(int i = 0; i < numDetections; i++) {
                apriltag_detection_t* det = nullptr;
                zarray_get(detections, i, &det);
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

        logger->trace("Detected {} april tags", zarray_size(detections));
        out.send(aprilTags);
        passthroughInputImage.send(inFrame);
    }
}

}  // namespace node
}  // namespace dai
