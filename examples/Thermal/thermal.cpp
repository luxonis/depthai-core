#include <cstdio>
#include <iostream>

#include "utility.hpp"

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"
#include "stdio.h"

static int mouseX, mouseY = 0;

void mouseCallback(int event, int x, int y, int flags, void* userdata) {
    mouseX = x;
    mouseY = y;
}

int main() {
    // Create pipeline
    dai::Pipeline pipeline;

    auto thermal = pipeline.create<dai::node::Thermal>();
    thermal->properties.baudrate = 256 * 192 * 2 * 8 * 25;
    // auto camera = pipeline.create<dai::node::ColorCamera>();
    // auto camera = pipeline.create<dai::node::MonoCamera>();
    auto xlink = pipeline.create<dai::node::XLinkOut>();
    auto xlinkRaw = pipeline.create<dai::node::XLinkOut>();
    // camera->setCamera("left");
    // camera->setBoardSocket(dai::CameraBoardSocket::CAM_D);
    thermal->out.link(xlink->input);
    thermal->raw.link(xlinkRaw->input);
    xlinkRaw->setStreamName("thermal_raw");
    xlink->setStreamName("thermal");
    dai::Device d(pipeline);
    auto q = d.getOutputQueue("thermal", 8, false);
    auto q_raw = d.getOutputQueue("thermal_raw", 8, false);

    const char* tempWindow = "temperature";
    const char *imageWindow = "image";

    cv::namedWindow(tempWindow);
    cv::setMouseCallback(tempWindow, mouseCallback);
    cv::namedWindow(imageWindow);
    cv::setMouseCallback(imageWindow, mouseCallback);
    cv::resizeWindow(tempWindow, 256, 192);
    cv::resizeWindow(imageWindow, 256, 192);
    while(true) {
        auto temp = q_raw->tryGet<dai::ImgFrame>();
        if(temp) {
            auto frame = temp->getCvFrame();
            cv::Mat frameFp32(temp->getHeight(), temp->getWidth(), CV_32F);
            frame.convertTo(frameFp32, CV_32F);
            cv::Mat normalized;
            cv::normalize(frameFp32, normalized, 0, 255, cv::NORM_MINMAX, CV_8UC1);
            cv::Mat colormapped(temp->getHeight(), temp->getWidth(), CV_8UC3);
            cv::applyColorMap(normalized, colormapped, cv::COLORMAP_MAGMA);

            // Draw crosshair
            cv::line(colormapped, cv::Point(mouseX - 10, mouseY), cv::Point(mouseX + 10, mouseY), cv::Scalar(0, 0, 0), 1);
            cv::line(colormapped, cv::Point(mouseX, mouseY - 10), cv::Point(mouseX, mouseY + 10), cv::Scalar(0, 0, 0), 1);
            // Draw deg C
            char text[32];
            sprintf(text, "%.1f deg C", frameFp32.at<float>(mouseY, mouseX));
            cv::putText(colormapped, text, cv::Point(mouseX + 10, mouseY - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
            cv::imshow(tempWindow, colormapped);
        }
        auto image = q->tryGet<dai::ImgFrame>();
        if(image) {
            cv::imshow(imageWindow, image->getCvFrame());
        }
        int key = cv::waitKey(1);
        if(key == 'q') {
            break;
        }
    }

    return 0;
}
