#include <cstdio>
#include <iostream>

#include "utility.hpp"

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

int main() {
    // Create pipeline
    dai::Pipeline pipeline;

    auto thermal = pipeline.create<dai::node::Thermal>();
    thermal->properties.baudrate = 256 * 192 * 2 * 8 * 10;
    // auto camera = pipeline.create<dai::node::ColorCamera>();
    // auto camera = pipeline.create<dai::node::MonoCamera>();
    auto xlink = pipeline.create<dai::node::XLinkOut>();
    auto xlink_raw = pipeline.create<dai::node::XLinkOut>();
    // camera->setCamera("left");
    // camera->setBoardSocket(dai::CameraBoardSocket::CAM_D);
    thermal->out.link(xlink->input);
    thermal->raw.link(xlink_raw->input);
    xlink_raw->setStreamName("thermal_raw");
    xlink->setStreamName("thermal");
    dai::Device d(pipeline);
    auto q = d.getOutputQueue("thermal", 8, false);
    auto q_raw = d.getOutputQueue("thermal_raw", 8, false);
    while(true) {
        auto img = q->tryGet<dai::ImgFrame>();
        if(img) {
            auto frame = img->getData();
            printf("DATA size: %d\n", frame.size()); // 98304 is the YUV422 size of RGB888i 192x256 = 147456
            cv::imshow("thermal", img->getCvFrame());
        }
        // Save the frame to disk as th.yuv

        // FILE* fp = fopen("th.yuv", "wb");
        // fwrite(frame.data(), 1, frame.size(), fp);
        // fclose(fp);
        // exit(0);
        // Convert to BGR888i
        // cv::Mat bgr(cv::Size(192, 256), CV_8UC3);
        // cv::cvtColor(cv::Mat(192, 256, CV_8UC2, frame.data()), bgr, cv::COLOR_YUV2BGR_YUY2);
        // printf("BGR frame type: %d, dimensions %d %d %d\n", bgr.type(), bgr.size().width, bgr.size().height, bgr.channels());
        // cv::imshow("thermal", bgr);

        auto img_raw = q_raw->tryGet<dai::ImgFrame>();
        if(img_raw) {
            auto frame = img_raw->getData();
            printf("RAW DATA size: %d\n", frame.size());
            cv::imshow("thermal_raw", img_raw->getCvFrame());
        }
        int key = cv::waitKey(1);
        if(key == 'q') {
            break;
        }
    }

    return 0;
}
