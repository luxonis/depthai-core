// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"
#include "xtensor/xsort.hpp"

class Face2ImageManipConfig : public dai::NodeCRTP<dai::node::ThreadedHostNode, Face2ImageManipConfig> {
public:
    Input inputDetections = dai::Node::Input{*this, {}};
    Output outputManipulators = dai::Node::Output{*this, {}};
    
    void run() override {
        while(isRunning()) {
            std::shared_ptr<dai::ImgDetections> inDet;
            inDet = inputDetections.get<dai::ImgDetections>();

            if(!inDet)
            {
                continue;
            }

            for(auto& detection : inDet->detections) {
              std::shared_ptr<dai::ImageManipConfig> manipulator = std::make_shared<dai::ImageManipConfig>();
                manipulator->setCropRect(detection.xmin,
                                        detection.ymin,
                                        detection.xmax,
                                        detection.ymax);
                manipulator->setResize(64,64);
                outputManipulators.send(manipulator);
            }
 
        }
    }
};

std::array<const char* const,5> emotions = {"neutral", "happy", "sad", "surprise", "anger"};

void displayFrame(cv::Mat& frame, std::vector<dai::ImgDetection>& detections, xt::xarray<float>& resultLayer)
{
    auto color = cv::Scalar(255, 0, 0);
    for(int i = 0; i < detections.size(); i++) {
        auto& detection = detections[i];
        int x1 = detection.xmin * frame.cols;
        int y1 = detection.ymin * frame.rows;
        int x2 = detection.xmax * frame.cols;
        int y2 = detection.ymax * frame.rows;
 
        std::stringstream confStr;
        auto emotionIndex = xt::argmax(resultLayer)(0);
        if(emotionIndex < emotions.size()) {
            confStr << emotions[emotionIndex];
        } else {
           confStr << "Err index: " << emotionIndex;
        }
        
        
        cv::putText(frame, confStr.str(), cv::Point(x1 + 10, y1 + 40), cv::FONT_HERSHEY_TRIPLEX, 0.5, color);
        cv::rectangle(frame, cv::Rect(cv::Point(x1, y1), cv::Point(x2, y2)), color, cv::FONT_HERSHEY_SIMPLEX);
    }
    cv::imshow("video", frame);
}

int main(int argc, char** argv) {
    std::string nnPath;
    std::string nnEmoPath;
    if(argc > 2) {
        nnPath = std::string(argv[1]);
        nnEmoPath = std::string(argv[2]);
    } else {
        std::cout << "call with arguments: {detection blob} {emotion blob}" << std::endl;
        return 1;
    }

  
    // Create pipeline
    auto device = std::make_shared<dai::Device>(dai::OpenVINO::VERSION_UNIVERSAL, dai::UsbSpeed::HIGH);
    dai::Pipeline pipeline(device);
    // Define source and output
    auto camRgb = pipeline.create<dai::node::ColorCamera>();
    auto nn = pipeline.create<dai::node::NeuralNetwork>();
    auto nnEmo = pipeline.create<dai::node::NeuralNetwork>();
    auto det = pipeline.create<dai::node::DetectionParser>();
    auto manipConf = pipeline.create<Face2ImageManipConfig>();
    auto manip = pipeline.create<dai::node::ImageManip>();
    
    // Camera props
    camRgb->setBoardSocket(dai::CameraBoardSocket::CAM_A);
    camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    camRgb->setPreviewSize(300, 300);
    camRgb->setInterleaved(false);

    // Face detection NN props
    nn->setNumInferenceThreads(2);
    nn->input.setBlocking(false);
    dai::OpenVINO::Blob blob1(nnPath);
    nn->setBlob(blob1);

    // Face detection NN parser props
    det->setBlob(blob1);
    det->setNNFamily(DetectionNetworkType::MOBILENET);
    det->setConfidenceThreshold(0.5);

    // Emotion detection NN props
    nnEmo->setNumInferenceThreads(2);
    nnEmo->input.setBlocking(false);
    dai::OpenVINO::Blob blob2(nnEmoPath);
    nnEmo->setBlob(blob2);
    
    // ImageManip props
    manip->initialConfig.setResize(64,64);

    // Linking
    /*
      rgb -> nn -> det -> manipConf -> manip -> nnEmo
          --------------------------->
    */
    camRgb->preview.link(nn->input);
    nn->out.link(det->input);
    det->out.link(manipConf->inputDetections);
    manipConf->outputManipulators.link(manip->inputConfig);
    camRgb->preview.link(manip->inputImage);
    manip->out.link(nnEmo->input);
    
    auto outPassthrough = nn->passthrough.createOutputQueue();
    auto outDet = det->out.createOutputQueue();
    auto outNNEmo = nnEmo->out.createOutputQueue();
    
    pipeline.start();
    while(pipeline.isRunning()) {
        std::shared_ptr<dai::ImgFrame> inRgb;
        std::shared_ptr<dai::ImgDetections> inDet;
        std::shared_ptr<dai::NNData> inNNEmo;
        
        inRgb = outPassthrough->get<dai::ImgFrame>();
        inDet = outDet->get<dai::ImgDetections>();
        inNNEmo = outNNEmo->get<dai::NNData>();
        
        cv::Mat frame;
        std::vector<dai::ImgDetection> detections;
        xt::xarray<float> resultLayer;
        
        if(inRgb) {
            frame = inRgb->getCvFrame();
        }

        if(inDet) {
            detections = inDet->detections;
        }

        if(inNNEmo && !inNNEmo->tensors.empty()) {
            resultLayer = inNNEmo->getTensor<float>(inNNEmo->tensors.back().name, false);
        }
        
        if(!frame.empty()) {
            displayFrame(frame, detections, resultLayer);
        }

        int key = cv::waitKey(1);
        if(key == 'q' || key == 'Q') {
            return 0;
        }
    }
    return 0;
}
