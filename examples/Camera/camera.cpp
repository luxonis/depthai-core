#include "depthai/depthai.hpp"

int main(int argc, char** args) {
    auto availableDevices = dai::Device::getAllAvailableDevices();
    auto connectedDevices = dai::Device::getAllConnectedDevices();

    for(auto& dev : availableDevices) {
        std::cout << "Available device: " << dev.toString() << std::endl;
    }

    for(auto& dev : connectedDevices) {
        std::cout << "Connected device: " << dev.toString() << std::endl;
    }

    dai::Device d;
    dai::Pipeline p;

    auto cam = p.create<dai::node::Camera>();
    cam->setBoardSocket(dai::CameraBoardSocket::CAM_A);
    cam->setMeshSource(dai::CameraProperties::WarpMeshSource::NONE);
    cam->setSize(4000, 4000);
    cam->setVideoSize(1920, 1080);
    cam->setPreviewSize(300, 300);
    // cam->setPreviewType(dai::ImgFrame::Type::RGB888i);
    cam->setVideoType(dai::ImgFrame::Type::NV12);

    // auto xout = p.create<dai::node::XLinkOut>();
    // xout->setStreamName("preview");
    // cam->preview.link(xout->input);

    auto xoutVid = p.create<dai::node::XLinkOut>();
    xoutVid->setStreamName("video");
    cam->video.link(xoutVid->input);

    d.startPipeline(p);

    // auto previewQ = d.getOutputQueue("preview");
    auto videoQ = d.getOutputQueue("video");
    while(1) {
        // auto preview = previewQ->tryGet<dai::ImgFrame>();
        // if(preview) {
        //     cv::imshow("preview", preview->getCvFrame());
        // }
        auto video = videoQ->tryGet<dai::ImgFrame>();
        if(video) {
            cv::imshow("video", video->getCvFrame());
        }
        if(cv::waitKey(1) == 'q') break;
    }
    return 0;
}
