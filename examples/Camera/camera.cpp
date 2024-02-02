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

    std::string ip = "192.168.102.110";
    // std::string ip = "192.168.197.55";
    dai::Device d(ip);
    dai::Pipeline p;

    // auto cam = p.create<dai::node::Camera>();
    // cam->setBoardSocket(dai::CameraBoardSocket::CAM_A);
    // // cam->setMeshSource(dai::CameraProperties::WarpMeshSource::NONE);
    // cam->setSize(1920, 1080);
    // // cam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    // cam->setVideoSize(1920/2, 1080/2);
    // // cam->setPreviewSize(300, 300);
    // cam->setFps(5.0);
    // // cam->setPreviewType(dai::ImgFrame::Type::RGB888i);
    // cam->setVideoType(dai::ImgFrame::Type::NV12);

    auto monoCam = p.create<dai::node::Camera>();
    // auto monoCam2 = p.create<dai::node::MonoCamera>();
    // auto monoCam = p.create<dai::node::MonoCamera>();
    monoCam->setBoardSocket(dai::CameraBoardSocket::CAM_C);
    // monoCam2->setBoardSocket(dai::CameraBoardSocket::CAM_B);
    int width = 1280;
    int height = 800;
    // int width = 640;
    // int height = 400;
    int previewNom = 1;
    int previewDenom = 4;
    monoCam->setResolution(dai::CameraProperties::SensorResolution::THE_1200_P);
    // monoCam2->setResolution(dai::MonoCameraProperties::SensorResolution::THE_1200_P);

    // monoCam2->setVideoSize(width-1, height-1);
    // monoCam->setVideoSize(width-1, height-1);
    // monoCam->setPreviewSize(width * previewNom / previewDenom, height * previewNom / previewDenom);
    // monoCam->setPreviewType(dai::ImgFrame::Type::GRAY8);
    // monoCam->setVideoType(dai::ImgFrame::Type::GRAY8);
    // monoCam->properties.sensorType = dai::CameraSensorType::MONO;

    auto monoXout = p.create<dai::node::XLinkOut>();
    monoXout->setStreamName("mono");
    // monoCam->out.link(monoXout->input);
    monoCam->video.link(monoXout->input);
    // monoCam2->raw.link(monoXout->input);

    auto xout = p.create<dai::node::XLinkOut>();
    xout->setStreamName("preview");
    // monoCam2->out.link(xout->input);
    // monoCam2->out.link(xout->input);

    auto xoutVid = p.create<dai::node::XLinkOut>();
    xoutVid->setStreamName("video");
    // cam->video.link(xoutVid->input);

    d.startPipeline(p);

    auto previewQ = d.getOutputQueue("preview", 4, false);
    auto videoQ = d.getOutputQueue("video", 4, false);
    auto monoQ = d.getOutputQueue("mono", 4, false);
    while(1) {
        auto preview = previewQ->tryGet<dai::ImgFrame>();
        if(preview) {
            cv::imshow("preview", preview->getCvFrame());
            printf("Preview WxW: %dx%d\n", preview->getWidth(), preview->getHeight());
        }
        auto video = videoQ->tryGet<dai::ImgFrame>();
        if(video) {
            cv::imshow("video", video->getCvFrame());
            printf("Video WxW: %dx%d\n", video->getWidth(), video->getHeight());
        }
        auto mono = monoQ->tryGet<dai::ImgFrame>();
        if(mono) {
            cv::imshow("mono", mono->getCvFrame());
        }
        switch(cv::waitKey(1)) {
            case 'q':
                return 0;
        }
    }
    return 0;
}


/**
 * @brief 
 * CAMERA:
 * GST PROPS:
id: 0, width: 1920, height: 1080, fps: 10.000000
orientation: 3, numPools: 3, v4l_cam: /dev/v4l-subdev1, v4l_sen: /dev/v4l-subdev0, v4l_lens: /dev/v4l-subdev2


GST PROPS:
id: 0, width: 1920, height: 1080, fps: 10.000000
orientation: 3, numPools: 3, v4l_cam: /dev/v4l-subdev1, v4l_sen: /dev/v4l-subdev0, v4l_lens: /dev/v4l-subdev2
stride 1, scanline 64
cam id:0, hflip:1, vflip:1

 */
