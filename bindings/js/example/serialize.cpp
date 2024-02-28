#include <iostream>

// include depthai library
#include <depthai/depthai.hpp>
#include <depthai/pipeline/datatype/StreamMessageParser.hpp>

// include opencv library (Optional, used only for the following example)
#include <opencv2/opencv.hpp>

using namespace std;

template <typename T>
void writeMessageToFile(const T& message, const char* filename) {
    const auto metadata = dai::StreamMessageParser::serializeMetadata(message);
    const auto data = message->getData();
    ofstream myfile;
    myfile.open(filename, ios::out | ios::binary);
    myfile.write(reinterpret_cast<const char*>(data.data()), data.size() * sizeof(data.front()));
    myfile.write(reinterpret_cast<const char*>(metadata.data()), metadata.size() * sizeof(metadata.front()));
    myfile.close();
}

int main() {
    // Create pipeline
    dai::Pipeline pipeline;
    auto colorCam = pipeline.create<dai::node::ColorCamera>();
    auto xlinkOut = pipeline.create<dai::node::XLinkOut>();
    xlinkOut->setStreamName("preview");
    colorCam->setInterleaved(true);
    colorCam->preview.link(xlinkOut->input);

    try {
        // Try connecting to device and start the pipeline
        dai::Device device(pipeline);

        // Get output queue
        auto preview = device.getOutputQueue("preview");

        cv::Mat frame;
        while(true) {
            // Receive 'preview' frame from device
            auto imgFrame = preview->get<dai::ImgFrame>();
            std::vector<std::uint8_t> data;
            writeMessageToFile(imgFrame, "img_frame.dump");

            // Show the received 'preview' frame
            cv::imshow("preview", cv::Mat(imgFrame->getHeight(), imgFrame->getWidth(), CV_8UC3, imgFrame->getData().data()));

            // Wait and check if 'q' pressed
            if(cv::waitKey(1) == 'q') return 0;
        }
    } catch(const std::runtime_error& err) {
        std::cout << err.what() << std::endl;
        return 1;
    }

    return 0;
}

