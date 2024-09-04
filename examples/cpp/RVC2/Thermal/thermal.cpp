/*
    Thermal camera example.
    Streams the temperature image from a thermal sensor and
    displays it with a crosshair with the temperature value at that point.
*/
#include "depthai/depthai.hpp"

volatile int mouseX, mouseY = 0;
void mouseCallback(int event, int x, int y, int flags, void* userdata) {
    mouseX = x;
    mouseY = y;
}

const cv::Scalar WHITE(255, 255, 255);

int main(int argc, char** args) {
    dai::Pipeline pipeline(true);
    auto thermal = pipeline.create<dai::node::Thermal>();
    auto outTemp = thermal->temperature.createOutputQueue();
    pipeline.start();

    const char* TEMP_WINDOW = "Thermal";
    cv::namedWindow(TEMP_WINDOW, cv::WINDOW_NORMAL);
    cv::setMouseCallback(TEMP_WINDOW, mouseCallback);
    const int WINDOW_SCALE_MULTIPLIER = 4;
    bool initialWindowResizeDone = false;
    while(true) {
        auto tempImgFrame = outTemp->get<dai::ImgFrame>();
        // On OAK-T the thermal stream is rather tiny, so we scale it up for better visualization.
        if(!initialWindowResizeDone) {
            cv::resizeWindow(TEMP_WINDOW, tempImgFrame->getWidth() * WINDOW_SCALE_MULTIPLIER, tempImgFrame->getHeight() * WINDOW_SCALE_MULTIPLIER);
            initialWindowResizeDone = true;
        }
        // Convert the FP16 image frame to CV_32F.
        cv::Mat frameFp32(tempImgFrame->getHeight(), tempImgFrame->getWidth(), CV_32F);
        tempImgFrame->getCvFrame().convertTo(frameFp32, CV_32F);
        // Colorize the temperature frame.
        cv::Mat normalized;
        cv::normalize(frameFp32, normalized, 0, 255, cv::NORM_MINMAX, CV_8UC1);
        cv::Mat colormapped(tempImgFrame->getHeight(), tempImgFrame->getWidth(), CV_8UC3);
        cv::applyColorMap(normalized, colormapped, cv::COLORMAP_MAGMA);
        if(mouseX < 0 || mouseY < 0 || mouseX >= colormapped.cols || mouseY >= colormapped.rows) {
            mouseX = std::max(0, std::min(static_cast<int>(mouseX), colormapped.cols - 1));
            mouseY = std::max(0, std::min(static_cast<int>(mouseY), colormapped.rows - 1));
        }
        double min, max;
        cv::minMaxLoc(frameFp32, &min, &max);
        const cv::Scalar textColor(255, 255, 255);
        ;
        // Draw crosshair
        cv::line(colormapped, cv::Point(mouseX - 10, mouseY), cv::Point(mouseX + 10, mouseY), textColor, 1);
        cv::line(colormapped, cv::Point(mouseX, mouseY - 10), cv::Point(mouseX, mouseY + 10), textColor, 1);
        // Draw deg C
        char text[32];
        snprintf(text, sizeof(text), "%.1f deg C", frameFp32.at<float>(mouseY, mouseX));
        bool putTextLeft = mouseX > colormapped.cols / 2;
        cv::putText(colormapped, text, cv::Point(putTextLeft ? mouseX - 100 : mouseX + 10, mouseY - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, textColor, 1);
        cv::imshow(TEMP_WINDOW, colormapped);
        // Can add controls for the sensor in the future here.
        int key = cv::waitKey(1);
        if(key == 'q') {
            break;
        }
    }

    return 0;
}
