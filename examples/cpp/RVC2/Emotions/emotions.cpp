// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

void displayFrame(cv::Mat& frame, std::vector<dai::ImgDetection>& detections)
{
    auto color = cv::Scalar(255, 0, 0);
    for(auto& detection : detections) {
        int x1 = detection.xmin * frame.cols;
        int y1 = detection.ymin * frame.rows;
        int x2 = detection.xmax * frame.cols;
        int y2 = detection.ymax * frame.rows;
 
        std::stringstream confStr;
        confStr << "label: " << detection.label << " " <<std::fixed << std::setprecision(2) << detection.confidence * 100;
        cv::putText(frame, confStr.str(), cv::Point(x1 + 10, y1 + 40), cv::FONT_HERSHEY_TRIPLEX, 0.5, color);
        cv::rectangle(frame, cv::Rect(cv::Point(x1, y1), cv::Point(x2, y2)), color, cv::FONT_HERSHEY_SIMPLEX);
    }
    cv::imshow("video", frame);
}

int main(int argc, char** argv) {
    std::string nnPath;
    if(argc > 1) {
        nnPath = std::string(argv[1]);
    } else {
        std::cout << "Provide path to face detection blob as first arg!" << std::endl;
        return 1;
    }

  
    // Create pipeline
    auto device = std::make_shared<dai::Device>(dai::OpenVINO::VERSION_UNIVERSAL, dai::UsbSpeed::HIGH);
    dai::Pipeline pipeline(device);
    // Define source and output
    auto camRgb = pipeline.create<dai::node::ColorCamera>();
    auto nn = pipeline.create<dai::node::NeuralNetwork>();
    auto det = pipeline.create<dai::node::DetectionParser>();
    
    // Properties
    camRgb->setBoardSocket(dai::CameraBoardSocket::CAM_A);
    camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    camRgb->setPreviewSize(300, 300);
    camRgb->setInterleaved(false);

    nn->setNumInferenceThreads(2);
    nn->input.setBlocking(false);
    dai::OpenVINO::Blob blob(nnPath);
    nn->setBlob(blob);
    
    det->setBlob(blob);
    det->setNNFamily(DetectionNetworkType::MOBILENET);
    det->setConfidenceThreshold(0.5);

    camRgb->preview.link(nn->input);
    nn->out.link(det->input);

    auto outPassthrough = nn->passthrough.createOutputQueue();
    auto outDet = det->out.createOutputQueue();
    
    pipeline.start();
    while(pipeline.isRunning()) {
        std::shared_ptr<dai::ImgFrame> inRgb;
        std::shared_ptr<dai::ImgDetections> inDet;
      
        inRgb = outPassthrough->get<dai::ImgFrame>();
        inDet = outDet->get<dai::ImgDetections>();

        cv::Mat frame;
        std::vector<dai::ImgDetection> detections;

        if(inRgb) {
            frame = inRgb->getCvFrame();
        }

        if(inDet) {
            detections = inDet->detections;
        }

        if(!frame.empty()) {
            displayFrame(frame, detections);
        }

        int key = cv::waitKey(1);
        if(key == 'q' || key == 'Q') {
            return 0;
        }
    }
    return 0;
}
