#include <atomic>
#include <csignal>
#include <iostream>
#include <memory>
#include <string>
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

// Custom host node for image annotations
class ImgAnnotationsGenerator : public dai::NodeCRTP<dai::node::HostNode, ImgAnnotationsGenerator> {
   public:
    Input& inputDet = inputs["detections"];
    Output& output = out;

    std::vector<std::string> labelMap;

    std::shared_ptr<ImgAnnotationsGenerator> build(Output& detections) {
        detections.link(inputDet);
        return std::static_pointer_cast<ImgAnnotationsGenerator>(this->shared_from_this());
    }

    std::shared_ptr<dai::Buffer> processGroup(std::shared_ptr<dai::MessageGroup> in) override {
        auto nnData = in->get<dai::ImgDetections>("detections");
        auto detections = nnData->detections;

        auto imgAnnt = std::make_shared<dai::ImgAnnotations>();
        imgAnnt->setTimestamp(nnData->getTimestamp());

        dai::ImgAnnotation annotation;
        for(const auto& detection : detections) {
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

            // Get label text
            std::string labelText;
            try {
                labelText = labelMap[detection.label];
            } catch(...) {
                labelText = std::to_string(detection.label);
            }

            text->text = labelText + " " + std::to_string(static_cast<int>(detection.confidence * 100)) + "%";
            text->fontSize = 50.5f;
            text->textColor = dai::Color(0.5f, 0.5f, 1.0f, 1.0f);
            text->backgroundColor = dai::Color(1.0f, 1.0f, 0.5f, 1.0f);

            annotation.points.push_back(*pointsAnnotation);
            annotation.texts.push_back(*text);
        }

        imgAnnt->annotations.push_back(annotation);
        return imgAnnt;
    }
};

int main(int argc, char** argv) {
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
        auto outputToEncode = cameraNode->requestOutput(std::make_pair(1920, 1440), dai::ImgFrame::Type::NV12);

        auto encoder = pipeline.create<dai::node::VideoEncoder>();
        encoder->setDefaultProfilePreset(30, dai::VideoEncoderProperties::Profile::MJPEG);

        // Linking
        outputToEncode->link(encoder->input);
        detectionNetwork->out.link(imageAnnotationsGenerator->inputDet);

        // Set label map
        imageAnnotationsGenerator->labelMap = detectionNetwork->getClasses().value();

        // Add remote connector topics
        remoteConnector.addTopic("encoded", encoder->out, "images");
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