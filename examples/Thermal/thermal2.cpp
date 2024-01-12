#include <unistd.h>

#include <chrono>
#include <algorithm>

#include "stdio.h"
#include "utility.hpp"

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

static volatile int mouseX, mouseY = 0;

void mouseCallback(int event, int x, int y, int flags, void* userdata) {
    mouseX = x;
    mouseY = y;
}

const cv::Scalar WHITE(255, 255, 255);

int main() {
    int fps = 0;
    int fpsCounter = 0;
    // Create pipeline
    dai::Pipeline pipeline;
    // auto color = pipeline.create<dai::node::Camera>();
    // color->setBoardSocket(dai::CameraBoardSocket::CAM_A);
    // color->setFps(25);
    // color->setVideoSize(1920, 1080);
    // color->setPreviewSize(1920 / 3, 1080 / 3);

    auto thermal = pipeline.create<dai::node::Camera>();
    thermal->setBoardSocket(dai::CameraBoardSocket::CAM_E);
    // thermal->properties.interleaved = false;
    thermal->setFps(25);
    int width = 256;
    int height = 192;
    thermal->setPreviewSize(width, height);
    thermal->setVideoSize(width, height);

    // auto mono = pipeline.create<dai::node::Camera>();
    // mono->setBoardSocket(dai::CameraBoardSocket::CAM_B);
    // mono->setPreviewSize(340, 200);

    // auto color2 = pipeline.create<dai::node::Camera>();
    // color2->setBoardSocket(dai::CameraBoardSocket::CAM_D);
    // color2->setPreviewSize(340, 200);

    auto xlink = pipeline.create<dai::node::XLinkOut>();
    auto xlink_raw = pipeline.create<dai::node::XLinkOut>();
    thermal->preview.link(xlink->input);
    // thermal->raw.link(xlink_raw->input);
    auto xlink_thermal_video = pipeline.create<dai::node::XLinkOut>();
    xlink_thermal_video->setStreamName("thermal_video");
    // thermal->video.link(xlink_thermal_video->input);

    // COLOR XLINK
    auto xlink_color = pipeline.create<dai::node::XLinkOut>();
    xlink_color->setStreamName("preview");
    // color->preview.link(xlink_color->input);
    auto xlink_color2 = pipeline.create<dai::node::XLinkOut>();
    xlink_color2->setStreamName("color2");
    // color2->preview.link(xlink_color2->input);

    // MONO XLINK
    auto xlink_mono = pipeline.create<dai::node::XLinkOut>();
    // mono->properties.interleaved = false;
    xlink_mono->setStreamName("preview_mono");
    // mono->preview.link(xlink_mono->input);

    xlink_raw->setStreamName("thermal_raw");
    xlink->setStreamName("thermal");
    dai::DeviceInfo info;
    info.mxid = "1944301031BA762700";  // 1944301031BA762700 (THE OPEN ONE), FULLY ASSEMBLED UNIT: 19443010B19D762700, FFC4p: 18443010811F5B1200
    dai::Device d(info);

    d.startPipeline(pipeline);
    auto q = d.getOutputQueue("thermal", 2, false);
    auto q_raw = d.getOutputQueue("thermal_raw", 2, false);
    auto q_color = d.getOutputQueue("preview", 2, false);
    auto q_color2 = d.getOutputQueue("color2", 2, false);
    auto q_mono = d.getOutputQueue("preview_mono", 2, false);
    auto q_thermal_video = d.getOutputQueue("thermal_video", 2, false);

    const char* tempWindow = "temperature";
    const char* imageWindow = "image";
    cv::namedWindow(tempWindow, cv::WINDOW_NORMAL);
    cv::setMouseCallback(tempWindow, mouseCallback);
    cv::namedWindow(imageWindow, cv::WINDOW_NORMAL);
    cv::setMouseCallback(imageWindow, mouseCallback);
    // Scale 4x and position one next to another
    cv::moveWindow(tempWindow, 0, 0);
    cv::resizeWindow(tempWindow, width * 4, height * 4);
    cv::moveWindow(imageWindow, width * 4, 0);
    cv::resizeWindow(imageWindow, width * 4, height * 4);
    auto startT = std::chrono::steady_clock::now();
    while(true) {
        auto temp = q_raw->tryGet<dai::ImgFrame>();
        auto currentTime = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - startT);
        if(elapsed > std::chrono::seconds(1)) {
            fps = fpsCounter;
            fpsCounter = 0;
            startT = currentTime;
            printf("FPS: %d\n", fps);
        }
        if(temp) {
            auto frame = temp->getCvFrame();
            cv::Mat frameFp32(temp->getHeight(), temp->getWidth(), CV_32F);
            frame.convertTo(frameFp32, CV_32F);
            cv::Mat normalized;
            cv::normalize(frameFp32, normalized, 0, 255, cv::NORM_MINMAX, CV_8UC1);
            cv::Mat colormapped(temp->getHeight(), temp->getWidth(), CV_8UC3);
            cv::applyColorMap(normalized, colormapped, cv::COLORMAP_MAGMA);
            if(mouseX < 0 || mouseY < 0 || mouseX >= colormapped.cols || mouseY >= colormapped.rows) {
                mouseX = std::max(0, std::min(static_cast<int>(mouseX), colormapped.cols - 1));
                mouseY = std::max(0, std::min(static_cast<int>(mouseY), colormapped.rows - 1));
            }
            float temperatureMeasured = frameFp32.at<float>(mouseY, mouseX);
            double min, max;
            cv::minMaxLoc(frameFp32, &min, &max);
            auto textColor = WHITE;  //- WHITE * (temperatureMeasured - min) / (max - min);
            // Draw crosshair
            cv::line(colormapped, cv::Point(mouseX - 10, mouseY), cv::Point(mouseX + 10, mouseY), textColor, 1);
            cv::line(colormapped, cv::Point(mouseX, mouseY - 10), cv::Point(mouseX, mouseY + 10), textColor, 1);
            // Draw deg C
            char text[32];
            sprintf(text, "%.1f deg C", frameFp32.at<float>(mouseY, mouseX));
            bool putTextLeft = mouseX > colormapped.cols / 2;
            cv::putText(colormapped, text, cv::Point(putTextLeft ? mouseX - 100 : mouseX + 10, mouseY - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, textColor, 1);
            cv::imshow(tempWindow, colormapped);
        }
        auto image = q->tryGet<dai::ImgFrame>();
        if(image) {
            fpsCounter++;
            cv::imshow(imageWindow, image->getCvFrame());
        }
        auto thermal_video = q_thermal_video->tryGet<dai::ImgFrame>();
        if(thermal_video) {
            cv::imshow("thermal_video", thermal_video->getCvFrame());
        }
        auto frColor = q_color->tryGet<dai::ImgFrame>();
        if(frColor) {
            cv::imshow("color", frColor->getCvFrame());
        }
        auto frColor2 = q_color2->tryGet<dai::ImgFrame>();
        if(frColor2) {
            cv::imshow("COLORTWO", frColor2->getCvFrame());
        }
        auto mono = q_mono->tryGet<dai::ImgFrame>();
        if(mono) {
            cv::imshow("mono", mono->getCvFrame());
        }

        int key = cv::waitKey(1);
        if(key == 'q') {
            break;
        }
    }

    return 0;
}
