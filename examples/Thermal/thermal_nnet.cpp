#include "depthai/depthai.hpp"
#include <vector>
#include <iostream>


int main(int argc, char **args) {
    dai::Device d("1944301031BA762700"); // 1944301031BA762700 (THE OPEN ONE), FULLY ASSEMBLED UNIT: 19443010B19D762700, FFC4p: 18443010811F5B1200

    dai::Pipeline p;
    auto thermalCam = p.create<dai::node::Camera>();
    thermalCam->setFps(15);
    thermalCam->setBoardSocket(dai::CameraBoardSocket::CAM_E);
    thermalCam->setPreviewSize(256, 192);
    thermalCam->setVideoSize(256, 192);

    // auto xlinkThermal = p.create<dai::node::XLinkOut>();
    // xlinkThermal->setStreamName("thermal");
    // thermalCam->raw.link(xlinkThermal->input);

    dai::OpenVINO::Blob blob("./examples/Thermal/thermal_net_sim_p.blob");

    // auto nnet = p.create<dai::node::NeuralNetwork>();
    auto nnet = p.create<dai::node::YoloDetectionNetwork>();
    nnet->setBlob(blob);
    nnet->setNumClasses(1);
    nnet->setCoordinateSize(4);
    nnet->setConfidenceThreshold(0.999);
    thermalCam->raw.link(nnet->input);
    auto xout_nnet = p.create<dai::node::XLinkOut>();
    xout_nnet->setStreamName("nn");
    nnet->out.link(xout_nnet->input);
    auto xout_thermal = p.create<dai::node::XLinkOut>();
    xout_thermal->setStreamName("thermal");
    nnet->passthrough.link(xout_thermal->input);

    // auto detParser = p.create<dai::node::DetectionParser>();
    // detParser->setBlob(blob);
    // detParser->setNNFamily(DetectionNetworkType::YOLO);
    // nnet->out.link(detParser->input);
    // auto xout_det = p.create<dai::node::XLinkOut>();
    // xout_det->setStreamName("detParser");
    // detParser->out.link(xout_det->input);
    d.startPipeline(p);

    auto q = d.getOutputQueue("nn", 2, false);
    auto qThermal = d.getOutputQueue("thermal", 2, false);
    // auto qDet = d.getOutputQueue("detParser", 2, false);
    while (true) {
        auto nnData = q->get<dai::ImgDetections>();
        // auto nnData = qDet->get<dai::ImgDetections>();
        // auto nnData = q->get<dai::NNData>();
        auto thermalPacket = qThermal->get<dai::ImgFrame>();
        auto frame = thermalPacket->getCvFrame();
        cv::Mat frameFp32(thermalPacket->getHeight(), thermalPacket->getWidth(), CV_32F);
        frame.convertTo(frameFp32, CV_32F);
        printf("PX: %f\n", frameFp32.at<float>(1, 1));
        cv::Mat normalized;
        cv::normalize(frameFp32, normalized, 0, 255, cv::NORM_MINMAX, CV_8UC1);

        cv::Mat colormapped(thermalPacket->getHeight(), thermalPacket->getWidth(), CV_8UC3);
        cv::applyColorMap(normalized, colormapped, cv::COLORMAP_MAGMA);

        // std::vector<cv::Rect> boxes;
        // std::vector<float> scores;
        // auto nnOut = nnData->getLayerFp16("output1_yolov6r2");
        // printf("nnOut size: %lu\n", nnOut.size());
        // dai::TensorInfo::DataType type;
        // nnData->getLayerDatatype("output1_yolov6r2", type);
        // printf("OUT DATA TYPE: %d\n", type);
        // int j=0;
        // for (int i=0; i<nnOut.size(); i+=86) {
        //     float score = nnOut[i+4];
        //     if (score > 0.5) {
        //         printf("NN: %f, %f, %f, %f, %f, %f %f, ", nnOut[i], nnOut[i+1], nnOut[i+2], nnOut[i+3], nnOut[i+4], nnOut[i+5]);
        //         for (int j=0; j<80; j++) {
        //             printf("%f, ", nnOut[i+5+j]);
        //         }
        //         printf("\n");
        //         int centerX = nnOut[i] * 256;
        //         int centerY = nnOut[i+1] * 192;
        //         int width = nnOut[i+2] * 256;
        //         int height = nnOut[i+3] * 192;
        //         boxes.push_back(cv::Rect(centerX - width / 2, centerY - height / 2, width, height));
        //         scores.push_back(score);
        //         break;
        //     }
        // }
        // std::vector<int> indices;
        // cv::dnn::NMSBoxes(boxes, scores, 0.5, 0.4, indices);
        // printf("Indices: %lu\n", indices.size());
        // for (int i=0; i<indices.size(); i++) {
        //     int idx = indices[i];
        //     cv::rectangle(colormapped, boxes[idx], cv::Scalar(0, 255, 0), 2);
        // }
        printf("detections: %lu\n", nnData->detections.size());
        for (auto &det : nnData->detections) {
            printf("XMIN: %f, YMIN: %f, XMAX: %f, YMAX: %f\n", det.xmin, det.ymin, det.xmax, det.ymax);
            cv::rectangle(colormapped, cv::Rect(det.xmin * 256, det.ymin * 192, (det.xmax - det.xmin) * 256, (det.ymax - det.ymin) * 192), cv::Scalar(0, 255, 0), 2);
        }

        // Display detections on thermal image
        // std::cout << "Layers:\n";
        // for (auto& layerName : nnData->getAllLayerNames()) {
        //     std::cout << layerName << ", ";
        // }
        // std::cout << "\n";

        // dai::TensorInfo tensor;
        // nnData->getLayer("output", tensor);
        // // auto detections = nnData->getLayerFp16("output");
        // printf("Tensor dims:");
        // for (auto &dim : tensor.dims) {
        //     std::cout << dim << ", ";
        // }



        // printf("Detections: %lu\n", detections.size());

        // printf("Detection0: %f %f %f %f %f %f\n", detections[0], detections[1], detections[2], detections[3], detections[4], detections[5]);
        
        cv::imshow("thermal", colormapped);
        if (cv::waitKey(1) == 'q') {
            break;
        }
    }



    return 0;
}
