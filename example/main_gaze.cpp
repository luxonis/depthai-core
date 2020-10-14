
#include <iostream>
#include <vector>
#include <array>
#include <tuple>

#include "depthai/Device.hpp"
#include "depthai/xlink/XLinkConnection.hpp"

#include "depthai-shared/Assets.hpp"

#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/ColorCamera.hpp"
#include "depthai/pipeline/node/XLinkOut.hpp"
#include "depthai/pipeline/node/NeuralNetwork.hpp"
#include "depthai/pipeline/node/XLinkIn.hpp"
#include "depthai/pipeline/node/MyProducer.hpp"
#include "depthai/pipeline/node/VideoEncoder.hpp"


#include "depthai-shared/datatype/NNData.hpp"
#include "depthai-shared/datatype/ImgFrame.hpp"

#include "opencv2/opencv.hpp"
#include "fp16/fp16.h"


std::shared_ptr<dai::DataInputQueue> frame_in = nullptr;
std::shared_ptr<dai::DataOutputQueue> face_nn = nullptr;
std::shared_ptr<dai::DataInputQueue> land_in = nullptr;
std::shared_ptr<dai::DataOutputQueue> land_nn = nullptr;


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

cv::Mat getPaddedROI(const cv::Mat &input, int top_left_x, int top_left_y, int width, int height, cv::Scalar paddingColor = cv::Scalar(0,0,0)) {
    int bottom_right_x = top_left_x + width;
    int bottom_right_y = top_left_y + height;

    cv::Mat output;
    if (top_left_x < 0 || top_left_y < 0 || bottom_right_x > input.cols || bottom_right_y > input.rows) {
        // border padding will be required
        int border_left = 0, border_right = 0, border_top = 0, border_bottom = 0;

        if (top_left_x < 0) {
            width = width + top_left_x;
            border_left = -1 * top_left_x;
            top_left_x = 0;
        }
        if (top_left_y < 0) {
            height = height + top_left_y;
            border_top = -1 * top_left_y;
            top_left_y = 0;
        }
        if (bottom_right_x > input.cols) {
            width = width - (bottom_right_x - input.cols);
            border_right = bottom_right_x - input.cols;
        }
        if (bottom_right_y > input.rows) {
            height = height - (bottom_right_y - input.rows);
            border_bottom = bottom_right_y - input.rows;
        }

        cv::Rect R(top_left_x, top_left_y, width, height);
        cv::copyMakeBorder(input(R), output, border_top, border_bottom, border_left, border_right, cv::BORDER_CONSTANT, paddingColor);
    }
    else {
        // no border padding required
        cv::Rect R(top_left_x, top_left_y, width, height);
        output = input(R);
    }
    return output;
}

cv::Mat getPaddedROI(const cv::Mat &input, cv::Rect roi, cv::Scalar paddingColor = cv::Scalar(0,0,0)) {
    return getPaddedROI(input, roi.x, roi.y, roi.width, roi.height, paddingColor);
}




dai::Pipeline createPipeline(std::string nnFolder){


    dai::Pipeline pipeline;
    
    // NeuralNetwork - Face
    auto frame_in = pipeline.create<dai::node::XLinkIn>();
    auto face_nn = pipeline.create<dai::node::NeuralNetwork>();
    auto face_nn_xout = pipeline.create<dai::node::XLinkOut>();

    frame_in->setStreamName("frame_in");
    face_nn->setBlobPath(nnFolder + "/models/face-detection-retail-0004/face-detection-retail-0004.blob");
    face_nn_xout->setStreamName("face_nn");

    frame_in->out.link(face_nn->input);
    face_nn->out.link(face_nn_xout->input);
        


    // NeuralNetwork - Landmarks
    auto land_nn = pipeline.create<dai::node::NeuralNetwork>();
    auto land_nn_xin = pipeline.create<dai::node::XLinkIn>();
    auto land_nn_xout = pipeline.create<dai::node::XLinkOut>();
    
    land_nn->setBlobPath(nnFolder + "/models/landmarks-regression-retail-0009/landmarks-regression-retail-0009.blob");
    land_nn_xin->setStreamName("landmark_in");
    land_nn_xout->setStreamName("landmark_nn");

    land_nn_xin->out.link(land_nn->input);
    land_nn->out.link(land_nn_xout->input);

    return pipeline;
}


void startPipeline(dai::Pipeline p){

    
    // CONNECT TO DEVICE
    bool found;
    dai::DeviceInfo deviceInfo;
    std::tie(found, deviceInfo) = dai::XLinkConnection::getFirstDevice(X_LINK_BOOTED);

    dai::Device device(deviceInfo);

    device.startPipeline(p);
    frame_in = device.getInputQueue("frame_in", 1);
    face_nn = device.getOutputQueue("face_nn", 1);
    land_in = device.getInputQueue("landmark_in", 1);
    land_nn = device.getOutputQueue("landmark_nn", 1);


}


struct Detection {
    unsigned int label;
    float score;
    float x_min;
    float y_min;
    float x_max;
    float y_max;
};
std::vector<Detection> parseDetectionsFp16(const std::vector<std::uint8_t>& data){
    
    std::vector<Detection> dets;

    auto result = reinterpret_cast<const std::uint16_t*>(data.data());

    int i = 0;
    while (/*valid*/fp16_ieee_to_fp32_value(result[i*7]) != -1.0f && i < 100)
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
    return dets;
}



std::tuple<cv::Mat, cv::Rect> runFace(cv::Mat frame, cv::Mat debugFrame){
    
    auto buff = std::make_shared<dai::NNData>();
    dai::TensorInfo info;
    info.offset = 0;
    buff->tensors = {info};

    cv::Mat detectorFrame;
    cv::resize(frame, detectorFrame, cv::Size(300,300));
    toPlanar(detectorFrame, buff->data);

    frame_in->send(buff);
    auto nnData = face_nn->get();

    auto dets = parseDetectionsFp16(nnData->data);
    int width = frame.cols;
    int height = frame.rows;
    
    cv::Rect rect{0,0,0,0};
    bool faceDetected = false;
    float maxScore = 0.0;
    for(const auto& det : dets){
        if(det.score > maxScore) maxScore = det.score;

        if(det.score > 0.4){
            faceDetected = true;
            rect = cv::Rect(cv::Point(det.x_min * width, det.y_min * height), cv::Point(det.x_max * width, det.y_max * height));
            cv::rectangle(debugFrame, rect, cv::Scalar(20, 255, 20), 2);

            break;
        }
    }
    
    cv::Mat headImage(frame, rect);

    return {headImage, rect};

}


std::tuple<cv::Mat, cv::Mat, cv::Point> runLandmark(cv::Mat faceFrame, cv::Rect faceRect, cv::Mat debugFrame){

    auto buff = std::make_shared<dai::RawBuffer>();
     
    cv::Mat nnFaceFrame;
    cv::resize(faceFrame, nnFaceFrame, cv::Size(48,48));
    toPlanar(nnFaceFrame, buff->data);

    land_in->send(buff);
    auto landData = land_nn->get();


    // parse landmarks
    std::array<float, 10> landmarks;
    auto resultFp16 = reinterpret_cast<const std::uint16_t*>(landData->data.data());
    for(unsigned int i = 0; i < landmarks.size(); i++) {
        landmarks[i] = fp16_ieee_to_fp32_value(resultFp16[i]);
    }

    int w = faceFrame.cols;
    int h = faceFrame.rows;
    auto rightEye = cv::Point(landmarks[0] * w, landmarks[1] * h);
    auto leftEye = cv::Point(landmarks[2]* w, landmarks[3] * h);
    auto nose = cv::Point(landmarks[4]* w, landmarks[5] * h);

    cv::Rect rightEyeRoi( cv::Point(rightEye.x - 30, rightEye.y - 30), cv::Point(rightEye.x + 30, rightEye.y + 30) );
    cv::Rect leftEyeRoi( cv::Point(leftEye.x - 30, leftEye.y - 30), cv::Point(leftEye.x + 30, leftEye.y + 30) );

    auto rightEyeImage = getPaddedROI(faceFrame, rightEyeRoi);
    auto leftEyeImage = getPaddedROI(faceFrame, leftEyeRoi);

    cv::circle(cv::Mat(debugFrame, faceRect), nose, 2, cv::Scalar(0,255,0), 5, 8, 0);
    cv::rectangle(cv::Mat(debugFrame, faceRect), rightEyeRoi, cv::Scalar(0,0,254), 2);
    cv::rectangle(cv::Mat(debugFrame, faceRect), leftEyeRoi, cv::Scalar(0,0,254), 2);

    return {rightEyeImage, leftEyeImage, nose};

}


int main(int argc, char** argv){

    if(argc <= 1){
        std::cout << "Pass path to project folder..." << std::endl;
    } else {
        std::string nnPath(argv[1]);
        cv::VideoCapture vid(nnPath + "/demo.mp4");
        cv::Mat frame, debugFrame;

        auto pipeline = createPipeline(nnPath);
        startPipeline(pipeline);

        while(true){
            vid >> frame;
            if(frame.empty()) break;

            frame.copyTo(debugFrame);
            
            cv::Mat faceFrame;
            cv::Rect frameRect;

            std::tie(faceFrame, frameRect) = runFace(frame, debugFrame);
            if(!frameRect.empty()) runLandmark(faceFrame, frameRect, debugFrame);

            cv::imshow("debug", debugFrame);
            for(int i = 0; i < 10; i++) cv::waitKey(1);


        }
     
        
    }


}
