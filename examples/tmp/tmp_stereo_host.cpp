#include <iostream>

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

int main(int argc, char**argv) {

    std::string leftPath;
    std::string rightPath;
    
    // Options
    bool usage = false, read = true, clear = false;
    std::string path = "";
    if(argc == 3) {
        leftPath = argv[1];
        rightPath = argv[2];
        std::cout << "Left path: " << leftPath << std::endl;
        std::cout << "Right path: " << rightPath << std::endl;

    } else {
        usage = true;
    }
    if(usage) {
        std::cout << "Usage: " << argv[0] << " [left rectified image 1280x720] [right rectified image 1280x720]" << std::endl;
        return -1;
    }

    cv::Mat cvinLeft = cv::imread(leftPath, cv::IMREAD_GRAYSCALE);
    if(cvinLeft.empty()) {
        std::cerr << "Left image path invalid!" << std::endl;
    }
    cv::Mat cvinRight = cv::imread(rightPath, cv::IMREAD_GRAYSCALE);
    if(cvinRight.empty()) {
        std::cerr << "Right image path invalid!" << std::endl;
    }

    cv::imshow("left input", cvinLeft);
    cv::imshow("right input", cvinRight);
    cv::waitKey(1);

    // Create pipeline
    dai::Pipeline pipeline;

    // Define sources and outputs
    auto monoLeft = pipeline.create<dai::node::XLinkIn>();
    auto monoRight = pipeline.create<dai::node::XLinkIn>();
    auto xoutLeft = pipeline.create<dai::node::XLinkOut>();
    auto xoutRight = pipeline.create<dai::node::XLinkOut>();
    auto xoutDepth = pipeline.create<dai::node::XLinkOut>();
    auto stereo = pipeline.create<dai::node::StereoDepth>();

    monoLeft->setStreamName("inLeft");
    monoRight->setStreamName("inRight");
    xoutLeft->setStreamName("left");
    xoutRight->setStreamName("right");
    xoutDepth->setStreamName("depth");

    monoLeft->setMaxDataSize(1280 * 720);
    monoRight->setMaxDataSize(1280 * 720);


    // Linking
    monoLeft->out.link(stereo->left);
    monoRight->out.link(stereo->right);
    stereo->syncedLeft.link(xoutLeft->input);
    stereo->syncedRight.link(xoutRight->input);
    stereo->depth.link(xoutDepth->input);
    stereo->setInputResolution(1280,720);
    
    // Connect to device and start pipeline
    dai::Device device(pipeline);


    auto qInLeft = device.getInputQueue("inLeft");
    auto qInRight = device.getInputQueue("inRight");
    auto qLeft = device.getOutputQueue("left", 4, false);
    auto qRight = device.getOutputQueue("right", 4, false);
    auto qDepth = device.getOutputQueue("depth", 4, false);

    int cnt = 0;

    while(true) {
        
        auto ingImgLeft = std::make_shared<dai::ImgFrame>();
        ingImgLeft->setData(std::vector<uint8_t>(cvinLeft.data, cvinLeft.data+cvinLeft.rows*cvinLeft.cols));
        ingImgLeft->setInstanceNum((unsigned int)dai::CameraBoardSocket::LEFT);
        ingImgLeft->setType(dai::ImgFrame::Type::RAW8);
        ingImgLeft->setWidth(1280);
        ingImgLeft->setHeight(720);
        qInLeft->send(ingImgLeft);

        auto ingImgRight = std::make_shared<dai::ImgFrame>();
        ingImgRight->setData(std::vector<uint8_t>(cvinRight.data, cvinRight.data+cvinRight.rows*cvinRight.cols));
        ingImgRight->setInstanceNum((unsigned int)dai::CameraBoardSocket::RIGHT);
        ingImgRight->setType(dai::ImgFrame::Type::RAW8);
        ingImgRight->setWidth(1280);
        ingImgRight->setHeight(720);
        qInRight->send(ingImgRight);

        auto inLeft = qLeft->get<dai::ImgFrame>();
        auto frameLeft = inLeft->getCvFrame();
        cv::imshow("left", frameLeft);

        auto inRight = qRight->get<dai::ImgFrame>();
        auto frameRight = inRight->getCvFrame();
        cv::imshow("right", frameRight);

        auto inDepth = qDepth->get<dai::ImgFrame>();
        auto frameDepth = inDepth->getCvFrame();
        cv::imshow("depth", frameDepth);

        cv::Mat disp; 
        frameDepth.convertTo(disp, CV_8UC1, 1.f/32);
        cv::imshow("disp", disp);

        cnt++;

        int key = cv::waitKey(1);
        if(key == 'q' || key == 'Q') {
            return 0;
        }
    }
    return 0;
}
