#include <iostream>
#include <cstdio>
#include <deque>

#include "utility.hpp"

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

constexpr float CAMERA_FPS = 60;

int main(int argc, char** argv){
    using namespace std;
    using namespace std::chrono;

    // Default blob path provided by Hunter private data download
    // Applicable for easier example usage only
    std::string nnPath(BLOB_PATH);
    // If path to blob specified, use that
    if(argc > 1){
        nnPath = std::string(argv[1]);
    }

    dai::Pipeline pipeline;

    auto colorCam = pipeline.create<dai::node::ColorCamera>();
    auto nn = pipeline.create<dai::node::MobileNetDetectionNetwork>();
    auto camOut = pipeline.create<dai::node::XLinkOut>();
    auto passthroughMeta = pipeline.create<dai::node::XLinkOut>();
    auto resultOut = pipeline.create<dai::node::XLinkOut>();

    camOut->setStreamName("preview");
    passthroughMeta->setMetadataOnly(true);
    passthroughMeta->setStreamName("passthroughMeta");
    resultOut->setStreamName("resultOut");
    
    // ColorCamera options
    colorCam->setPreviewSize(300, 300);
    colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    colorCam->setInterleaved(false);
    colorCam->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);
    colorCam->setFps(CAMERA_FPS);

    // NN input options
    nn->input.setBlocking(false);
    nn->input.setQueueSize(1);
    nn->setBlobPath(nnPath);
    // Change to 1 to observe slower consumer syncing
    nn->setNumInferenceThreads(2);


    // Link nodes CAM -> XLINK
    colorCam->preview.link(nn->input);
    colorCam->preview.link(camOut->input);
    nn->passthrough.link(passthroughMeta->input); 
    nn->out.link(resultOut->input); 

    // Connect to device and start pipeline
    dai::Device device(pipeline);
    device.startPipeline();

    // Create input & output queues
    auto previewQueue = device.getOutputQueue("preview");
    auto passthroughQueue = device.getOutputQueue("passthroughMeta");
    auto resultQueue = device.getOutputQueue("resultOut");

    // Get first passthrough and result at the same time
    auto prevPassthrough = passthroughQueue->get<dai::ImgFrame>();
    auto prevResult = resultQueue->get<dai::ImgDetections>();

    // statistics
    nanoseconds sumLatency{0};
    milliseconds avgLatency{0};
    int numFrames = 0;
    int nnFps = 0, lastNnFps = 0;
    int camFps = 0, lastCamFps = 0;
    auto lastTime = steady_clock::now();

    std::mutex queueMtx;
    std::deque<cv::Mat> queue;
    std::atomic<bool> running{true};

    // Displaying thread
    std::thread displayThread([&running, &queueMtx, &queue](){
        using namespace std::chrono;
        cv::Mat frame;
        while(running){
            auto t1 = steady_clock::now();
            {
                std::unique_lock<std::mutex> l(queueMtx);
                if(!queue.empty()){
                    frame = queue.front();
                    queue.pop_front();
                }
            }
            if(!frame.empty()){
                cv::imshow("frame", frame);
            }
            auto toSleep = (seconds(1) / CAMERA_FPS) - (steady_clock::now() - t1);
            if(toSleep > milliseconds(0)){
                if(cv::waitKey(duration_cast<milliseconds>(toSleep).count()) == 'q'){
                    running = false;
                }
            }            
        }
    });

    while(running){

        // Get first passthrough and result at the same time
        auto passthrough = passthroughQueue->get<dai::ImgFrame>();
        auto result = resultQueue->get<dai::ImgDetections>();

        nnFps++;

        // Match up prevResults and previews
        while(true){
            
            // pop the preview
            auto preview = previewQueue->get<dai::ImgFrame>();
            camFps++;

            // process
            cv::Mat frame = toMat(preview->getData(), preview->getWidth(), preview->getHeight(), 3, 1);

            // Draw all detections
            for(const auto& d : prevResult->detections){
                int x1 = d.xmin * frame.cols;
                int y1 = d.ymin * frame.rows;
                int x2 = d.xmax * frame.cols;
                int y2 = d.ymax * frame.rows;
                cv::rectangle(frame, cv::Rect(cv::Point(x1, y1), cv::Point(x2, y2)), cv::Scalar(255,255,255));
            }

            // Draw avg latency and previous fps
            cv::putText(frame, std::string("NN: ") + std::to_string(lastNnFps), cv::Point(5,20), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255,0,0));
            cv::putText(frame, std::string("Cam: ") + std::to_string(lastCamFps), cv::Point(5,50), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255,0,0));
            cv::putText(frame, std::string("Latency: ") + std::to_string(avgLatency.count()), cv::Point(frame.cols - 120, 20), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255,0,0));

            // Send to be displayed
            {
                std::unique_lock<std::mutex> l(queueMtx);
                queue.push_back(std::move(frame));
            }
            
            // If seq number >= next detection seq number - 1, break
            if(preview->getSequenceNum() >= prevPassthrough->getSequenceNum() - 1){

                numFrames++;
                sumLatency = sumLatency + (steady_clock::now() - preview->getTimestamp());

                if(steady_clock::now() - lastTime >= seconds(1)){
                    
                    // calculate fps
                    lastNnFps = nnFps;
                    nnFps = 0;

                    // calculate cam fps
                    lastCamFps = camFps;
                    camFps = 0;

                    //calculate latency
                    avgLatency = duration_cast<milliseconds>(sumLatency / numFrames);
                    sumLatency = nanoseconds(0);
                    numFrames = 0;
                    
                    //reset last time
                    lastTime = steady_clock::now();
                }

                // break out of the loop
                break;
            }
        }

        // Move current NN results to prev
        prevPassthrough = passthrough;
        prevResult = result;

    }

    displayThread.join();

}
