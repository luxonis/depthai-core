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
    // Create pipeline
    dai::Pipeline pipeline;

    // auto thermal = pipeline.create<dai::node::Camera>();
    // thermal->setBoardSocket(dai::CameraBoardSocket::CAM_E);
    // thermal->properties.interleaved = false;
    // thermal->setPreviewSize(256, 192);

    auto color = pipeline.create<dai::node::Camera>();
    color->setBoardSocket(dai::CameraBoardSocket::CAM_A);
    color->setPreviewSize(340, 200);

    auto mono = pipeline.create<dai::node::Camera>();
    mono->setBoardSocket(dai::CameraBoardSocket::CAM_B);
    mono->setPreviewSize(340, 200);

    auto xlink = pipeline.create<dai::node::XLinkOut>();
    auto xlink_raw = pipeline.create<dai::node::XLinkOut>();
    // thermal->preview.link(xlink->input);
    // thermal->raw.link(xlink_raw->input);

    auto xlink_color = pipeline.create<dai::node::XLinkOut>();
    xlink_color->setStreamName("preview");
    color->preview.link(xlink_color->input);
    auto xlink_mono = pipeline.create<dai::node::XLinkOut>();
    mono->properties.interleaved = false;
    xlink_mono->setStreamName("preview_mono");
    mono->preview.link(xlink_mono->input);

    xlink_raw->setStreamName("thermal_raw");
    xlink->setStreamName("thermal");
    dai::Device d(pipeline);
    auto q = d.getOutputQueue("thermal", 8, false);
    auto q_raw = d.getOutputQueue("thermal_raw", 8, false);
    auto q_color = d.getOutputQueue("preview", 8, false);
    auto q_mono = d.getOutputQueue("preview_mono", 8, false);

    const char* tempWindow = "temperature";
    const char* imageWindow = "image";
    cv::namedWindow(tempWindow, cv::WINDOW_NORMAL);
    cv::setMouseCallback(tempWindow, mouseCallback);
    cv::namedWindow(imageWindow, cv::WINDOW_NORMAL);
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

            float temperatureMeasured = frameFp32.at<float>(mouseY, mouseX);
            double min, max;
            cv::minMaxLoc(frameFp32, &min, &max);
            auto textColor = WHITE - WHITE * (temperatureMeasured - min) / (max - min);
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
            // Convert from YUV422 to bgr
            auto frame = image->getCvFrame();
            printf("FRAME KIND: %d\n", image->getType());
            cv::Mat bgr;
            cv::cvtColor(cv::Mat(192, 256, CV_8UC2, image->getData().data()), bgr, cv::COLOR_YUV2BGR_YUY2);
            cv::imshow(imageWindow, bgr);
        }
        auto color = q_color->tryGet<dai::ImgFrame>();
        if(color) {
            cv::imshow("color", color->getCvFrame());
        }
        auto mono = q_mono->tryGet<dai::ImgFrame>();
        if(mono) {
            auto onePlane = std::vector<uint8_t>(mono->getData().begin(), mono->getData().begin() + mono->getWidth() * mono->getHeight());
            cv::Mat onePlaneMat(mono->getHeight(), mono->getWidth(), CV_8UC1, onePlane.data());
            cv::imshow("mono", onePlaneMat);
        }

        int key = cv::waitKey(1);
        if(key == 'q') {
            break;
        }
    }

    return 0;
}
