
#include <iostream>
#include <cstdio>

#include "utility.hpp"

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

static bool syncNN = true;

dai::Pipeline createNNPipeline(std::string nnPath){

    dai::Pipeline p;

    auto colorCam = p.create<dai::node::ColorCamera>();
    auto xlinkOut = p.create<dai::node::XLinkOut>();
    auto nn1 = p.create<dai::node::NeuralNetwork>();
    auto nnOut = p.create<dai::node::XLinkOut>();


    nn1->setBlobPath(nnPath);

    xlinkOut->setStreamName("preview");
    nnOut->setStreamName("detections");    

    colorCam->setPreviewSize(300, 300);
    colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    colorCam->setInterleaved(false);
    colorCam->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);

    // Link plugins CAM -> NN -> XLINK
    colorCam->preview.link(nn1->input);
    if (syncNN) nn1->passthrough.link(xlinkOut->input);
    else colorCam->preview.link(xlinkOut->input);

    nn1->out.link(nnOut->input);

    return p;

}



int main(int argc, char** argv){
    using namespace std;

    // Default blob path provided by Hunter private data download
    // Applicable for easier example usage only
    std::string nnPath(BLOB_PATH);
    
    // If path to blob specified, use that
    if(argc > 1){
        nnPath = std::string(argv[1]);
    }

    // Print which blob we are using
    printf("Using blob at path: %s\n", nnPath.c_str());

    // Create pipeline
    dai::Pipeline p = createNNPipeline(nnPath);

    // Connect to device with above created pipeline
    dai::Device d(p);
    // Start the pipeline
    d.startPipeline();

    cv::Mat frame;
    auto preview = d.getOutputQueue("preview");
    auto detections = d.getOutputQueue("detections");

    while(1){

        auto imgFrame = preview->get<dai::ImgFrame>();
        if(imgFrame){
            printf("Frame - w: %d, h: %d\n", imgFrame->getWidth(), imgFrame->getHeight());
            frame = toMat(imgFrame->getData(), imgFrame->getWidth(), imgFrame->getHeight(), 3, 1);
        }

        struct Detection {
            unsigned int label;
            float score;
            float x_min;
            float y_min;
            float x_max;
            float y_max;
        };

        vector<Detection> dets;

        auto det = detections->get<dai::NNData>();
        std::vector<float> detData = det->getFirstLayerFp16();
        if(detData.size() > 0){
            int i = 0;
            while (detData[i*7] != -1.0f) {
                Detection d;
                d.label = detData[i*7 + 1];
                d.score = detData[i*7 + 2];
                d.x_min = detData[i*7 + 3];
                d.y_min = detData[i*7 + 4];
                d.x_max = detData[i*7 + 5];
                d.y_max = detData[i*7 + 6];
                i++;
                dets.push_back(d);
            }
        }

        for(const auto& d : dets){
            int x1 = d.x_min * frame.cols;
            int y1 = d.y_min * frame.rows;
            int x2 = d.x_max * frame.cols;
            int y2 = d.y_max * frame.rows;

            cv::rectangle(frame, cv::Rect(cv::Point(x1, y1), cv::Point(x2, y2)), cv::Scalar(255,255,255));
        }

        printf("===================== %lu detection(s) =======================\n", dets.size());
        for (unsigned det = 0; det < dets.size(); ++det) {
            printf("%5d | %6.4f | %7.4f | %7.4f | %7.4f | %7.4f\n",
                    dets[det].label,
                    dets[det].score,
                    dets[det].x_min,
                    dets[det].y_min,
                    dets[det].x_max,
                    dets[det].y_max);
        }

        cv::imshow("preview", frame);
        int key = cv::waitKey(1);
        if (key == 'q'){
            return 0;
        } 
    }

    return 0;
}
