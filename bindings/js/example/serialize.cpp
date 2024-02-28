#include <iostream>

// include depthai library
#include <depthai/depthai.hpp>

// include opencv library (Optional, used only for the following example)
#include <opencv2/opencv.hpp>

using namespace std;

void writeDataToFile(const char* filename, const std::vector<std::uint8_t>& data) {
    ofstream myfile;
    myfile.open(filename, ios::out | ios::binary);
    myfile.write(reinterpret_cast<const char*>(data.data()), data.size() * sizeof(data.front()));
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
            dai::utility::serialize(*imgFrame, data);

            dai::ImgFrame imgFrame2;
            const auto rc = dai::utility::deserialize(data, imgFrame2);
            if(!rc) {
                std::cout << "Deserialization failed" << std::endl;
            }

            writeDataToFile("frame.dump", data);

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

