
#include <iostream>
#include <cstdio>

#include "utility.hpp"

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"


dai::Pipeline createMonoPipeline(){

    dai::Pipeline p;

    auto monoCam = p.create<dai::node::MonoCamera>();
    auto xlinkOut = p.create<dai::node::XLinkOut>();
    xlinkOut->setStreamName("mono");
    
    // Set camera socket
    monoCam->setBoardSocket(dai::CameraBoardSocket::RIGHT);

    // Link plugins CAM -> XLINK
    monoCam->out.link(xlinkOut->input);
    
    return p;

}


int main(){
    using namespace std;
    using namespace std::chrono;

    // Create pipeline
    dai::Pipeline p = createMonoPipeline();

    // Connect to device with above created pipeline
    dai::Device d(p);
    // Start the pipeline
    d.startPipeline();

    cv::Mat frame;
    auto monoQueue = d.getOutputQueue("mono");

    while(1){

        auto imgFrame = monoQueue->get<dai::ImgFrame>();
        if(imgFrame){

            int latencyMs = duration_cast<milliseconds>(steady_clock::now() - imgFrame->getTimestamp()).count();
            printf("Frame - w: %d, h: %d, latency %dms\n", imgFrame->getWidth(), imgFrame->getHeight(), latencyMs);

            frame = cv::Mat(imgFrame->getHeight(), imgFrame->getWidth(), CV_8UC1, imgFrame->getData().data());
            
            cv::imshow("video", frame);
            int key = cv::waitKey(1);
            if (key == 'q'){
                return 0;
            } 
        } else {
            std::cout << "Not ImgFrame" << std::endl;
        }
        
    }
    return 0;
}