#include <atomic>
#include <csignal>
#include <iostream>
#include <memory>
#include <vector>

#include "depthai/depthai.hpp"
#include "depthai/pipeline/datatype/ImgAnnotations.hpp"
#include "depthai/remote_connection/RemoteConnection.hpp"
// Global flag for graceful shutdown
std::atomic<bool> quitEvent(false);

// Signal handler
void signalHandler(int signum) {
    quitEvent = true;
}

// Extended detections class
class ImgDetectionsExtended : public dai::ImgDetections {
   public:
    ImgDetectionsExtended(const std::shared_ptr<dai::ImgDetections>& detections) {
        this->detections = detections->detections;
        this->setTimestamp(detections->getTimestamp());
    }

    std::shared_ptr<dai::Buffer> getVisualizationMessage() {
        auto imgAnnt = std::make_shared<dai::ImgAnnotations>();
        imgAnnt->setTimestamp(this->getTimestamp());

        auto annotation = std::make_shared<dai::ImgAnnotation>();

        for(const auto& detection : this->detections) {
            // Create points annotation for bounding box
            auto pointsAnnotation = std::make_shared<dai::PointsAnnotation>();
            pointsAnnotation->type = dai::PointsAnnotationType::LINE_STRIP;
            pointsAnnotation->points = {dai::Point2f(detection.xmin, detection.ymin),
                                        dai::Point2f(detection.xmax, detection.ymin),
                                        dai::Point2f(detection.xmax, detection.ymax),
                                        dai::Point2f(detection.xmin, detection.ymax)};

            // Set colors and thickness
            pointsAnnotation->outlineColor = dai::Color(1.0f, 0.5f, 0.5f, 1.0f);
            pointsAnnotation->fillColor = dai::Color(0.5f, 1.0f, 0.5f, 0.5f);
            pointsAnnotation->thickness = 2.0f;

            // Create text annotation
            auto text = std::make_shared<dai::TextAnnotation>();
            text->position = dai::Point2f(detection.xmin, detection.ymin);
            text->text = "Test annotation";
            text->fontSize = 50.5f;
            text->textColor = dai::Color(0.5f, 0.5f, 1.0f, 1.0f);
            text->backgroundColor = dai::Color(1.0f, 1.0f, 0.5f, 1.0f);

            annotation->points.push_back(*pointsAnnotation);
            annotation->texts.push_back(*text);
        }

        imgAnnt->annotations.push_back(*annotation);
        return imgAnnt;
    }
};

// Custom host node for image annotations
class ImgAnnotationsGenerator : public dai::NodeCRTP<dai::node::HostNode, ImgAnnotationsGenerator> {
   public:
    Input& inputDet = inputs["detections"];
    Output& output = out;

    std::shared_ptr<ImgAnnotationsGenerator> build(Output& detections) {
        detections.link(inputDet);
        return std::static_pointer_cast<ImgAnnotationsGenerator>(this->shared_from_this());
    }

    std::shared_ptr<dai::Buffer> processGroup(std::shared_ptr<dai::MessageGroup> in) override {
        auto nnData = in->get<dai::ImgDetections>("detections");
        auto extended = std::make_shared<ImgDetectionsExtended>(nnData);
        return extended->getVisualizationMessage();
    }
};

int main() {
    // Set up signal handlers
    signal(SIGTERM, signalHandler);
    signal(SIGINT, signalHandler);

    try {
        // Create remote connection
        dai::RemoteConnection remoteConnector;

        // Create pipeline
        dai::Pipeline pipeline;

        // Create nodes
        auto cameraNode = pipeline.create<dai::node::Camera>();
        cameraNode->build(dai::CameraBoardSocket::CAM_A);

        auto detectionNetwork = pipeline.create<dai::node::DetectionNetwork>();
        dai::NNModelDescription modelDesc;
        modelDesc.model = "yolov6-nano";
        detectionNetwork->build(cameraNode, modelDesc);

        auto imageAnnotationsGenerator = pipeline.create<ImgAnnotationsGenerator>();
        auto outputToVisualize = cameraNode->requestOutput(std::make_pair(640, 480), dai::ImgFrame::Type::NV12);

        // Linking
        detectionNetwork->out.link(imageAnnotationsGenerator->inputDet);

        // Add remote connector topics
        remoteConnector.addTopic("encoded", *outputToVisualize, "images");
        remoteConnector.addTopic("detections", detectionNetwork->out, "images");
        remoteConnector.addTopic("annotations", imageAnnotationsGenerator->output, "images");

        // Start pipeline
        pipeline.start();

        // Register pipeline with remote connector
        remoteConnector.registerPipeline(pipeline);

        // Main loop
        while(pipeline.isRunning() && !quitEvent) {
            if(remoteConnector.waitKey(1) == 'q') {
                pipeline.stop();
                break;
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