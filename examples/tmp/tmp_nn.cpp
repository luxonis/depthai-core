
#include <iostream>

#include "utility.hpp"

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

static constexpr auto MODEL_IN_WIDTH = 512;
static constexpr auto MODEL_IN_HEIGHT = 512;
static constexpr auto MIN_SCORE = 0.3;

int main(int argc, char** argv) {
    // If path to blob specified, use that
    int camId = 0;
    if(argc > 1) {
        camId = std::stoi(argv[1]);
    }

    // Create pipeline
    dai::Pipeline pipeline;

    // Define sources and outputs
    auto nn = pipeline.create<dai::node::NeuralNetwork>();
    auto xin = pipeline.create<dai::node::XLinkIn>();
    auto xout = pipeline.create<dai::node::XLinkOut>();

    nn->setXmlModelPath(MODEL_XML_PATH, MODEL_BIN_PATH);

    xin->setStreamName("nn_in");
    xout->setStreamName("nn_out");

    xin->setMaxDataSize(MODEL_IN_WIDTH * MODEL_IN_HEIGHT * 3);
    xin->setNumFrames(4);

    // Linking
    xin->out.link(xout->input);
    nn->out.link(xout->input);

    // Open Webcam
    cv::VideoCapture webcam(camId);

    // Connect to device and start pipeline
    dai::Device device(pipeline);

    cv::Mat frame;
    auto in = device.getInputQueue("nn_in");
    auto detections = device.getOutputQueue("nn_out");

    using namespace std::chrono;
    auto startTime = steady_clock::now();
    int counter = 0;
    float fps = 0;

    while(true) {
        // data to send further
        auto tensor = std::make_shared<dai::RawImgFrame>();

        // Read frame from webcam
        webcam >> frame;

        // crop and resize
        cv::Mat resizedFrame = resizeKeepAspectRatio(frame, cv::Size(MODEL_IN_WIDTH, MODEL_IN_HEIGHT), cv::Scalar(0));

        toPlanar(resizedFrame, tensor->data);

        auto inputFrame = std::make_shared<dai::ImgFrame>(tensor);
        inputFrame->setType(dai::RawImgFrame::Type::BGR888p);
        inputFrame->setWidth(MODEL_IN_WIDTH);
        inputFrame->setHeight(MODEL_IN_HEIGHT);

        // tensor->data = std::vector<std::uint8_t>(frame.data, frame.data + frame.total());
        in->send(tensor);

        counter++;
        auto currentTime = steady_clock::now();
        auto elapsed = duration_cast<duration<float>>(currentTime - startTime);
        if(elapsed > seconds(1)) {
            fps = counter / elapsed.count();
            counter = 0;
            startTime = currentTime;
        }

        struct Detection {
            unsigned int label;
            float score;
            float x_min;
            float y_min;
            float x_max;
            float y_max;
        };

        std::vector<Detection> dets;
        auto outImg = detections->get<dai::ImgFrame>();
        auto cvframe = outImg->getCvFrame();
        std::stringstream fpsStr;
        fpsStr << "NN fps: " << std::fixed << std::setprecision(2) << fps;
        cv::putText(cvframe, fpsStr.str(), cv::Point(2, MODEL_IN_HEIGHT - 4), cv::FONT_HERSHEY_TRIPLEX, 0.4, cv::Scalar(255, 255, 255));

        auto det = detections->get<dai::NNData>();
        std::vector<float> detData = det->getFirstLayerFp16();
        if(detData.size() > 0) {
            int i = 0;
            while(detData[i * 7] != -1.0f) {
                Detection d;
                d.label = detData[i * 7 + 1];
                d.score = detData[i * 7 + 2];
                d.x_min = detData[i * 7 + 3];
                d.y_min = detData[i * 7 + 4];
                d.x_max = detData[i * 7 + 5];
                d.y_max = detData[i * 7 + 6];
                i++;
                if(d.score > MIN_SCORE) {
                    dets.push_back(d);
                }
            }
        }

        for(const auto& d : dets) {
            int x1 = d.x_min * resizedFrame.cols;
            int y1 = d.y_min * resizedFrame.rows;
            int x2 = d.x_max * resizedFrame.cols;
            int y2 = d.y_max * resizedFrame.rows;

            cv::rectangle(resizedFrame, cv::Rect(cv::Point(x1, y1), cv::Point(x2, y2)), cv::Scalar(255, 255, 255));
        }

        printf("===================== %lu detection(s) =======================\n", static_cast<unsigned long>(dets.size()));
        for(unsigned det = 0; det < dets.size(); ++det) {
            printf("%5d | %6.4f | %7.4f | %7.4f | %7.4f | %7.4f\n",
                   dets[det].label,
                   dets[det].score,
                   dets[det].x_min,
                   dets[det].y_min,
                   dets[det].x_max,
                   dets[det].y_max);
        }

        cv::imshow("preview", cvframe);

        int key = cv::waitKey(1);
        if(key == 'q') {
            return 0;
        }
    }
    return 0;
}
