/*
    Thermal camera example.
    Streams temperature and thermal image from thermal sensor and displays them.
*/
#include <algorithm>
#include <cstdio>

#include "depthai/depthai.hpp"

volatile int mouseX, mouseY = 0;
void mouseCallback(int event, int x, int y, int flags, void* userdata) {
    mouseX = x;
    mouseY = y;
}

const cv::Scalar WHITE(255, 255, 255);

int main() {
    dai::Device d;
    dai::Pipeline pipeline;
    auto thermal = pipeline.create<dai::node::Camera>();
    // Find the sensor width, height.
    int width, height;
    bool thermal_found = false;
    for(auto& features : d.getConnectedCameraFeatures()) {
        if(std::find_if(features.supportedTypes.begin(),
                        features.supportedTypes.end(),
                        [](const dai::CameraSensorType& type) { return type == dai::CameraSensorType::THERMAL; })
           != features.supportedTypes.end()) {
            thermal->setBoardSocket(features.socket);  // Thermal will always be on CAM_E
            width = features.width;
            height = features.height;
            thermal_found = true;
        }
    }
    if(!thermal_found) {
        throw std::runtime_error("Thermal camera not found!");
    }
    thermal->setPreviewSize(width, height);
    auto xlink = pipeline.create<dai::node::XLinkOut>();
    auto xlinkRaw = pipeline.create<dai::node::XLinkOut>();
    // Output preview,video, isp: RGB or NV12 or YUV420 thermal image.
    thermal->preview.link(xlink->input);
    // Output raw: FP16 temperature data (degrees Celsius)
    thermal->raw.link(xlinkRaw->input);

    xlinkRaw->setStreamName("thermal_raw");
    xlink->setStreamName("thermal");
    d.startPipeline(pipeline);
    auto q = d.getOutputQueue("thermal", 2, false);
    auto qRaw = d.getOutputQueue("thermal_raw", 2, false);

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
    while(true) {
        auto temp = qRaw->tryGet<dai::ImgFrame>();
        if(temp) {
            auto frame = temp->getCvFrame();
            // Retrieve one point of fp16 data
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
            double min, max;
            cv::minMaxLoc(frameFp32, &min, &max);
            auto textColor = WHITE;
            // Draw crosshair
            cv::line(colormapped, cv::Point(mouseX - 10, mouseY), cv::Point(mouseX + 10, mouseY), textColor, 1);
            cv::line(colormapped, cv::Point(mouseX, mouseY - 10), cv::Point(mouseX, mouseY + 10), textColor, 1);
            // Draw deg C
            char text[32];
            snprintf(text, sizeof(text), "%.1f deg C", frameFp32.at<float>(mouseY, mouseX));
            bool putTextLeft = mouseX > colormapped.cols / 2;
            cv::putText(colormapped, text, cv::Point(putTextLeft ? mouseX - 100 : mouseX + 10, mouseY - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, textColor, 1);
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
