#include <iostream>
#include <opencv2/opencv.hpp>

#include "depthai/depthai.hpp"

volatile int mouseX = 0, mouseY = 0;

void onMouse(int event, int x, int y, int flags, void* userdata) {
    mouseX = x;
    mouseY = y;
}

int main() {
    // Create pipeline
    dai::Pipeline pipeline(true);

    // Create nodes
    auto thermal = pipeline.create<dai::node::Thermal>();
    // Output raw: FP16 temperature data (degrees Celsius)
    auto qTemperature = thermal->temperature.createOutputQueue();
    // Output color: YUV422i image data
    auto qColor = thermal->color.createOutputQueue();

    // Start pipeline
    pipeline.start();

    const char* MAGMA_WINDOW_NAME = "Colorized Temperature";
    const char* IMAGE_WINDOW_NAME = "Thermal image";

    // Scale 4x and position one next to another
    cv::namedWindow(MAGMA_WINDOW_NAME, cv::WINDOW_NORMAL);
    cv::namedWindow(IMAGE_WINDOW_NAME, cv::WINDOW_NORMAL);
    bool initialRescaleAndPositionDone = false;

    while(true) {
        auto inTemperature = qTemperature->get<dai::ImgFrame>();
        auto inColor = qColor->get<dai::ImgFrame>();

        // temperature data is float16: convert it to float32
        cv::Mat thermalData(inTemperature->getHeight(), inTemperature->getWidth(), CV_32F);
        inTemperature->getCvFrame().convertTo(thermalData, CV_32F);

        // Normalize thermal data
        cv::Mat normalizedThermalData;
        cv::normalize(thermalData, normalizedThermalData, 0, 1, cv::NORM_MINMAX);
        normalizedThermalData.convertTo(normalizedThermalData, CV_8U, 255.0);

        // Apply color map
        cv::Mat colormappedFrame;
        cv::applyColorMap(normalizedThermalData, colormappedFrame, cv::COLORMAP_MAGMA);

        if(!initialRescaleAndPositionDone) {
            cv::moveWindow(MAGMA_WINDOW_NAME, 0, 0);
            int width = colormappedFrame.cols;
            int height = colormappedFrame.rows;
            cv::resizeWindow(MAGMA_WINDOW_NAME, width * 4, height * 4);
            cv::moveWindow(IMAGE_WINDOW_NAME, width * 4, 0);
            cv::resizeWindow(IMAGE_WINDOW_NAME, width * 4, height * 4);
            cv::setMouseCallback(MAGMA_WINDOW_NAME, onMouse);
            cv::setMouseCallback(IMAGE_WINDOW_NAME, onMouse);
            initialRescaleAndPositionDone = true;
        }

        // Clamp mouse coordinates
        mouseX = std::max(0, std::min(static_cast<int>(mouseX), thermalData.cols - 1));
        mouseY = std::max(0, std::min(static_cast<int>(mouseY), thermalData.rows - 1));

        // Draw crosshair
        cv::Scalar textColor(255, 255, 255);
        cv::line(colormappedFrame, cv::Point(mouseX - 10, mouseY), cv::Point(mouseX + 10, mouseY), textColor, 1);
        cv::line(colormappedFrame, cv::Point(mouseX, mouseY - 10), cv::Point(mouseX, mouseY + 10), textColor, 1);

        // Draw temperature text
        std::string text = std::to_string(thermalData.at<float>(mouseY, mouseX)) + " deg C";
        bool putTextLeft = mouseX > colormappedFrame.cols / 2;
        cv::putText(colormappedFrame, text, cv::Point(mouseX - (putTextLeft ? 100 : -10), mouseY - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, textColor, 1);

        cv::imshow(MAGMA_WINDOW_NAME, colormappedFrame);
        cv::imshow(IMAGE_WINDOW_NAME, inColor->getCvFrame());

        if(cv::waitKey(1) == 'q') {
            break;
        }
    }

    return 0;
}