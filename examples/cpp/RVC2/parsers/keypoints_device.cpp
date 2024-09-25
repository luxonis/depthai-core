#include <depthai/depthai.hpp>

int main() {
    dai::NNModelDescription modelDescription;
    modelDescription.modelSlug = "mediapipe-face-landmarker";
    modelDescription.modelVersionSlug = "192x192";
    modelDescription.platform = "RVC2";
    std::string archivePath = dai::getModelFromZoo(modelDescription, true);
    dai::NNArchive nnArchive(archivePath);

    dai::Pipeline pipeline;

    auto cam = pipeline.create<dai::node::Camera>()->build();
    auto largeOutput = cam->requestOutput(std::pair(720, 720), dai::ImgFrame::Type::BGR888p);

    auto manip = pipeline.create<dai::node::ImageManip>();
    manip->initialConfig.setResize(192, 192);
    largeOutput->link(manip->inputImage);

    auto nn = pipeline.create<dai::node::NeuralNetwork>()->build(manip->out, nnArchive);

    auto parser = pipeline.create<dai::node::KeypointsParser>()->build(nnArchive);
    nn->out.link(parser->input);

    auto videoQ = largeOutput->createOutputQueue();
    auto keypointsQ = parser->out.createOutputQueue();

    pipeline.start();

    while(pipeline.isRunning()) {
        auto frame = videoQ->get<dai::ImgFrame>();
        auto cvFrame = frame->getCvFrame();
        auto keypoints = keypointsQ->get<dai::Keypoints>()->keypoints;
        
        for (auto keypoint : keypoints) {
            int x = keypoint.x * frame->getWidth();
            int y = keypoint.y * frame->getHeight();
            cv::circle(cvFrame, cv::Point(x, y), 2, cv::Scalar(0, 255, 0), -1);
        }

        cv::imshow("Display", cvFrame);
        auto key = cv::waitKey(1);
        if(key == 'q' || key == 'Q') {
            break;
        }
    }

    return 0;
}
