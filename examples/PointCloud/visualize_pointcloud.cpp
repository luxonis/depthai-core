#include <iostream>
#include <opencv2/opencv.hpp>

#include "depthai-shared/common/CameraBoardSocket.hpp"
#include "depthai/depthai.hpp"

int main() {
    auto pipeline = dai::Pipeline();
    auto monoLeft = pipeline.create<dai::node::MonoCamera>();
    auto monoRight = pipeline.create<dai::node::MonoCamera>();
    auto color = pipeline.create<dai::node::ColorCamera>();
    auto depth = pipeline.create<dai::node::StereoDepth>();
    auto pointcloud = pipeline.create<dai::node::PointCloud>();
    auto imagemanip = pipeline.create<dai::node::ImageManip>();
    auto xout = pipeline.create<dai::node::XLinkOut>();
    auto xoutDepth = pipeline.create<dai::node::XLinkOut>();

    monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    monoLeft->setCamera("left");
    monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    monoRight->setCamera("right");

    color->setCamera("color");
    color->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);

    imagemanip->initialConfig.setFrameType(dai::ImgFrame::Type::RGB888p);
    imagemanip->setMaxOutputFrameSize(3072000);

    color->isp.link(imagemanip->inputImage);

    // Create a node that will produce the depth map (using disparity output as
    // it's easier to visualize depth this way)
    depth->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::HIGH_ACCURACY);
    // Options: MEDIAN_OFF, KERNEL_3x3, KERNEL_5x5, KERNEL_7x7 (default)
    depth->initialConfig.setMedianFilter(dai::MedianFilter::KERNEL_7x7);
    depth->setLeftRightCheck(true);
    depth->setExtendedDisparity(false);
    depth->setSubpixel(true);
    depth->setDepthAlign(dai::CameraBoardSocket::RGB);

    xout->setStreamName("out");
    xoutDepth->setStreamName("depth");

    monoLeft->out.link(depth->left);
    monoRight->out.link(depth->right);
    depth->depth.link(pointcloud->inputDepth);
    depth->disparity.link(xoutDepth->input);
    // imagemanip->out.link(pointcloud->inputColor);
    pointcloud->out.link(xout->input);

    auto viewer = std::make_unique<pcl::visualization::PCLVisualizer>("Cloud Viewer");
    bool first = true;

    dai::Device device(pipeline);

    auto q = device.getOutputQueue("out", 8, false);
    auto qDepth = device.getOutputQueue("depth", 8, false);
    long counter = 0;
    while(true) {
        std::cout << "Waiting for data" << std::endl;
        auto depthImg = qDepth->get<dai::ImgFrame>();
        cv::imshow("depth", depthImg->getFrame());
        cv::waitKey(1);
        auto pclMsg = q->get<dai::PointCloudData>();
        std::cout << "Got data" << std::endl;
        if(!pclMsg) {
            std::cout << "No data" << std::endl;
            continue;
        }
        auto points = pclMsg->getPointsXYZ();
        if(points.empty()) {
            std::cout << "Empty point cloud" << std::endl;
            continue;
        }
        float minx = 10e10, miny = 10e10, minz = 10e10, maxx = -10e10, maxy = -10e10, maxz = -10e10;
        uint8_t minr = 255, ming = 255, minb = 255, maxr = 0, maxg = 0, maxb = 0;
        // for (auto& point : points) {
        //     minx = std::min(minx, point.x);
        //     miny = std::min(miny, point.y);
        //     minz = std::min(minz, point.z);
        //     maxx = std::max(maxx, point.x);
        //     maxy = std::max(maxy, point.y);
        //     maxz = std::max(maxz, point.z);
        //     minr = std::min(minr, point.r);
        //     ming = std::min(ming, point.g);
        //     minb = std::min(minb, point.b);
        //     maxr = std::max(maxr, point.r);
        //     maxg = std::max(maxg, point.g);
        //     maxb = std::max(maxb, point.b);
        // }
        std::cout << "Number of points: " << points.size() / 3 << std::endl;
        std::cout << "Min x: " << pclMsg->getMinX() << " " << minx << std::endl;
        std::cout << "Min y: " << pclMsg->getMinY() << " " << miny << std::endl;
        std::cout << "Min z: " << pclMsg->getMinZ() << " " << minz << std::endl;
        std::cout << "Max x: " << pclMsg->getMaxX() << " " << maxx << std::endl;
        std::cout << "Max y: " << pclMsg->getMaxY() << " " << maxy << std::endl;
        std::cout << "Max z: " << pclMsg->getMaxZ() << " " << maxz << std::endl;

        std::cout << "Red: " << (int)minr << " " << (int)maxr << std::endl;
        std::cout << "Green: " << (int)ming << " " << (int)maxg << std::endl;
        std::cout << "Blue: " << (int)minb << " " << (int)maxb << std::endl;

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = pclMsg->toPclData();
        if(first) {
            viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud");
            first = false;
        } else {
            viewer->updatePointCloud(cloud, "cloud");
        }

        viewer->spinOnce(10);

        if(viewer->wasStopped()) {
            break;
        }
    }

    return 0;
}
