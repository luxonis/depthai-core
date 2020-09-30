#include <iostream>

#include "depthai/device.hpp"
#include "depthai/xlink/XLinkConnection.hpp"

#include "depthai-shared/Assets.hpp"

#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/ColorCamera.hpp"
#include "depthai/pipeline/node/XLinkOut.hpp"
#include "depthai/pipeline/node/NeuralNetwork.hpp"
#include "depthai/pipeline/node/XLinkIn.hpp"
#include "depthai/pipeline/node/MyProducer.hpp"


#include "depthai-shared/datatype/NNTensor.hpp"
#include "depthai-shared/datatype/ImgFrame.hpp"

#include "opencv2/opencv.hpp"
#include "fp16/fp16.h"



cv::Mat toMat(const std::vector<uint8_t>& data, int w, int h , int numPlanes, int bpp){
    
    cv::Mat frame;

    if(numPlanes == 3){
        frame = cv::Mat(h, w, CV_8UC3);

        // optimization (cache)
        for(int i = 0; i < w*h; i++) {
            uint8_t b = data.data()[i + w*h * 0];
            frame.data[i*3+0] = b;
        }
        for(int i = 0; i < w*h; i++) {                
            uint8_t g = data.data()[i + w*h * 1];    
            frame.data[i*3+1] = g;
        }
        for(int i = 0; i < w*h; i++) {
            uint8_t r = data.data()[i + w*h * 2];
            frame.data[i*3+2] = r;
        }
                    
    } else {
        if(bpp == 3){
            frame = cv::Mat(h, w, CV_8UC3);
            for(int i = 0; i < w*h*bpp; i+=3) {
                uint8_t b,g,r;
                b = data.data()[i + 2];
                g = data.data()[i + 1];
                r = data.data()[i + 0];    
                frame.at<cv::Vec3b>( (i/bpp) / w, (i/bpp) % w) = cv::Vec3b(b,g,r);
            }

        } else if(bpp == 6) {
            //first denormalize
            //dump
            
            frame = cv::Mat(h, w, CV_8UC3);
            for(int y = 0; y < h; y++){
                for(int x = 0; x < w; x++){

                    const uint16_t* fp16 = (const uint16_t*) (data.data() + (y*w+x)*bpp);                        
                    uint8_t r = (uint8_t) (fp16_ieee_to_fp32_value(fp16[0]) * 255.0f);
                    uint8_t g = (uint8_t) (fp16_ieee_to_fp32_value(fp16[1]) * 255.0f);
                    uint8_t b = (uint8_t) (fp16_ieee_to_fp32_value(fp16[2]) * 255.0f);
                    frame.at<cv::Vec3b>(y, x) = cv::Vec3b(b,g,r);
                }
            }
            
        }
    }

    return frame;
}


void toPlanar(cv::Mat& bgr, std::vector<std::uint8_t>& data){

    data.resize(bgr.cols * bgr.rows * 3);
    for(int y = 0; y < bgr.rows; y++){
        for(int x = 0; x < bgr.cols; x++){
            auto p = bgr.at<cv::Vec3b>(y,x);
            data[x + y*bgr.cols + 0 * bgr.rows*bgr.cols] = p[0];
            data[x + y*bgr.cols + 1 * bgr.rows*bgr.cols] = p[1];
            data[x + y*bgr.cols + 2 * bgr.rows*bgr.cols] = p[2];
        }
    }


/*
    std::vector<cv::Mat> planes(3);
    for(unsigned int i = 0; planes.size(); i++){ 
        cv::extractChannel(bgr, planes[i], i);
    }

    cv::Mat planarBgr;
    cv::vconcat(planes, planarBgr);

    data.clear();
    data.assign(planarBgr.data, planarBgr.data + planarBgr.total()*planarBgr.channels());
*/
}


cv::Mat resizeKeepAspectRatio(const cv::Mat &input, const cv::Size &dstSize, const cv::Scalar &bgcolor)
{
    cv::Mat output;

    double h1 = dstSize.width * (input.rows/(double)input.cols);
    double w2 = dstSize.height * (input.cols/(double)input.rows);
    if( h1 <= dstSize.height) {
        cv::resize( input, output, cv::Size(dstSize.width, h1));
    } else {
        cv::resize( input, output, cv::Size(w2, dstSize.height));
    }

    int top = (dstSize.height-output.rows) / 2;
    int down = (dstSize.height-output.rows+1) / 2;
    int left = (dstSize.width - output.cols) / 2;
    int right = (dstSize.width - output.cols+1) / 2;

    cv::copyMakeBorder(output, output, top, down, left, right, cv::BORDER_CONSTANT, bgcolor );

    return output;
}



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
    colorCam->setCamId(0);
    colorCam->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);

    // Link plugins CAM -> NN -> XLINK
    colorCam->preview.link(nn1->in);
    colorCam->preview.link(xlinkOut->in);
    nn1->out.link(nnOut->in);

    return p;

}

dai::Pipeline createCameraPipeline(){



    dai::Pipeline p;

    auto colorCam = p.create<dai::node::ColorCamera>();
    auto xlinkOut = p.create<dai::node::XLinkOut>();
    xlinkOut->setStreamName("preview");
    
    colorCam->setPreviewSize(300, 300);
    colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    colorCam->setInterleaved(true);
    colorCam->setCamId(0);

    // Link plugins CAM -> XLINK
    colorCam->preview.link(xlinkOut->in);
    
    return p;

}


dai::Pipeline createCameraFullPipeline(){



    dai::Pipeline p;

    auto colorCam = p.create<dai::node::ColorCamera>();
    auto xlinkOut = p.create<dai::node::XLinkOut>();
    xlinkOut->setStreamName("video");
    
    colorCam->setPreviewSize(300, 300);
    colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    colorCam->setInterleaved(true);
    colorCam->setCamId(0);

    // Link plugins CAM -> XLINK
    colorCam->video.link(xlinkOut->in);
    
    return p;

}

void startPreview(){

    dai::Pipeline p = createCameraPipeline();

    using namespace std;

    bool found;
    dai::DeviceInfo deviceInfo;
    std::tie(found, deviceInfo) = dai::XLinkConnection::getFirstDevice(X_LINK_BOOTED);

    if(found) {
        dai::Device d(deviceInfo);

        d.startPipeline(p);

        
        cv::Mat frame;
        auto preview = d.getOutputQueue("preview");

        while(1){

            auto imgFrame = preview->get<dai::ImgFrame>();
            if(imgFrame){

                printf("Frame - w: %d, h: %d\n", imgFrame->fb.width, imgFrame->fb.height);

                frame = cv::Mat(imgFrame->fb.height, imgFrame->fb.width, CV_8UC3, imgFrame->data.data());
                           
                cv::imshow("preview", frame);
                cv::waitKey(1);
            } else {
                std::cout << "Not ImgFrame" << std::endl;
            }
            
        }

    } else {
        cout << "No booted (debugger) devices found..." << endl;
    }

    
}

void startVideo(){

    dai::Pipeline p = createCameraFullPipeline();

    using namespace std;

    bool found;
    dai::DeviceInfo deviceInfo;
    std::tie(found, deviceInfo) = dai::XLinkConnection::getFirstDevice(X_LINK_BOOTED);

    if(found) {
        dai::Device d(deviceInfo);

        d.startPipeline(p);

        
        cv::Mat frame;
        auto preview = d.getOutputQueue("preview");
        auto detections = d.getOutputQueue("detections");



        while(1){

            auto imgFrame = preview->get<dai::ImgFrame>();
            if(imgFrame){

                printf("Frame - w: %d, h: %d\n", imgFrame->fb.width, imgFrame->fb.height);

                frame = cv::Mat(imgFrame->fb.height * 3 / 2, imgFrame->fb.width, CV_8UC1, imgFrame->data.data());
                
                cv::Mat rgb(imgFrame->fb.height, imgFrame->fb.width, CV_8UC3);

                cv::cvtColor(frame, rgb, cv::COLOR_YUV2BGR_NV12);
                
                cv::imshow("video", rgb);
                cv::waitKey(1);
            } else {
                std::cout << "Not ImgFrame" << std::endl;
            }
            
        }

    } else {
        cout << "No booted (debugger) devices found..." << endl;
    }


}


void startNN(std::string nnPath){
    using namespace std;

    dai::Pipeline p = createNNPipeline(nnPath);

    bool found;
    dai::DeviceInfo deviceInfo;
    std::tie(found, deviceInfo) = dai::XLinkConnection::getFirstDevice(X_LINK_BOOTED);

    if(found) {
        dai::Device d(deviceInfo);

        d.startPipeline(p);

        
        cv::Mat frame;
        auto preview = d.getOutputQueue("preview");
        auto detections = d.getOutputQueue("detections");

        while(1){

            auto imgFrame = preview->get<dai::ImgFrame>();
            if(imgFrame){

                printf("Frame - w: %d, h: %d\n", imgFrame->fb.width, imgFrame->fb.height);
                frame = toMat(imgFrame->data, imgFrame->fb.width, imgFrame->fb.height, 3, 1);

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

            auto det = detections->get<dai::NNTensor>();
            if(det){

                auto result = reinterpret_cast<std::uint16_t*>(det->data.data());

                int i = 0;
                while (/*valid*/fp16_ieee_to_fp32_value(result[i*7]) != -1.0f)
                {
                    Detection d;
                    d.label = fp16_ieee_to_fp32_value(result[i*7 + 1]);
                    d.score = fp16_ieee_to_fp32_value(result[i*7 + 2]);
                    d.x_min = fp16_ieee_to_fp32_value(result[i*7 + 3]);
                    d.y_min = fp16_ieee_to_fp32_value(result[i*7 + 4]);
                    d.x_max = fp16_ieee_to_fp32_value(result[i*7 + 5]);
                    d.y_max = fp16_ieee_to_fp32_value(result[i*7 + 6]);
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
            cv::waitKey(1);

        }

    } else {
        cout << "No booted (debugger) devices found..." << endl;
    }

}



void startWebcam(int camId, std::string nnPath){

    using namespace std;


    // CREATE PIPELINE
    dai::Pipeline p;

    auto xin = p.create<dai::node::XLinkIn>();
    //auto producer = p.create<dai::node::MyProducer>();
    auto nn = p.create<dai::node::NeuralNetwork>();
    auto xout = p.create<dai::node::XLinkOut>();


    //producer->setProcessor(dai::ProcessorType::LOS);
    nn->setBlobPath(nnPath);
    
    xin->setStreamName("nn_in");
    xin->setMaxDataSize(300*300*3);
    xin->setNumFrames(4);

    xout->setStreamName("nn_out");

    // Link plugins XLINK -> NN -> XLINK
    xin->out.link(nn->in);
    //producer->out.link(nn->in);

    nn->out.link(xout->in);



    // CONNECT TO DEVICE

    bool found;
    dai::DeviceInfo deviceInfo;
    std::tie(found, deviceInfo) = dai::XLinkConnection::getFirstDevice(X_LINK_BOOTED);

    if(found) {
        dai::Device d(deviceInfo);

        d.startPipeline(p);

        
        cv::VideoCapture webcam(camId);
    
        cv::Mat frame;
        auto in = d.getInputQueue("nn_in");
        auto detections = d.getOutputQueue("nn_out");

        while(1){
            
            // data to send further
            auto tensor = std::make_shared<dai::RawBuffer>();

            // Read frame from webcam
            webcam >> frame;

            // crop and resize
            frame = resizeKeepAspectRatio(frame, cv::Size(300,300), cv::Scalar(0));

            // transform to BGR planar 300x300
            toPlanar(frame, tensor->data);

            //tensor->data = std::vector<std::uint8_t>(frame.data, frame.data + frame.total());
            in->send(tensor);

            struct Detection {
                unsigned int label;
                float score;
                float x_min;
                float y_min;
                float x_max;
                float y_max;
            };

            vector<Detection> dets;

            auto det = detections->get<dai::NNTensor>();
            if(det){

                auto result = reinterpret_cast<std::uint16_t*>(det->data.data());

                int i = 0;
                while (/*valid*/fp16_ieee_to_fp32_value(result[i*7]) != -1.0f)
                {
                    Detection d;
                    d.label = fp16_ieee_to_fp32_value(result[i*7 + 1]);
                    d.score = fp16_ieee_to_fp32_value(result[i*7 + 2]);
                    d.x_min = fp16_ieee_to_fp32_value(result[i*7 + 3]);
                    d.y_min = fp16_ieee_to_fp32_value(result[i*7 + 4]);
                    d.x_max = fp16_ieee_to_fp32_value(result[i*7 + 5]);
                    d.y_max = fp16_ieee_to_fp32_value(result[i*7 + 6]);
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
            cv::waitKey(1);

        }

    } else {
        cout << "No booted (debugger) devices found..." << endl;
    }

}


void startTest(int id){

    using namespace std;

    // CONNECT TO DEVICE
    bool found;
    dai::DeviceInfo deviceInfo;
    std::tie(found, deviceInfo) = dai::XLinkConnection::getFirstDevice(X_LINK_BOOTED);

    if(found) {
        dai::Device d(deviceInfo);
        d.startTestPipeline(id);
    }

}


int main(int argc, char** argv){
    using namespace std;
    cout << "Hello World!" << endl;

    if(argc <= 1){
        std::cout << "Pass path to NN blob";
        return 0;
    }
    std::string nnPath(argv[1]);
 
    if(nnPath == "test0"){
        startTest(0);
    } else if(nnPath == "test1"){
        startTest(1);
    } else if(nnPath == "test2"){
        startTest(2);
    } else {
        startWebcam(0, nnPath);
    }


    return 0;
}