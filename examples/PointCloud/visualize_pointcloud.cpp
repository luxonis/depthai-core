#include <iostream>

#include "depthai/depthai.hpp"

int main() {
    auto pipeline = dai::Pipeline();
    auto monoLeft = pipeline.create<dai::node::MonoCamera>();
    auto monoRight = pipeline.create<dai::node::MonoCamera>();
    auto depth = pipeline.create<dai::node::StereoDepth>();
    auto pointcloud = pipeline.create<dai::node::PointCloud>();
    auto xout = pipeline.create<dai::node::XLinkOut>();

    monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    monoLeft->setCamera("left");
    monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    monoRight->setCamera("right");

    // Create a node that will produce the depth map (using disparity output as
    // it's easier to visualize depth this way)
    depth->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::HIGH_DENSITY);
    // Options: MEDIAN_OFF, KERNEL_3x3, KERNEL_5x5, KERNEL_7x7 (default)
    depth->initialConfig.setMedianFilter(dai::MedianFilter::KERNEL_7x7);
    depth->setLeftRightCheck(true);
    depth->setExtendedDisparity(false);
    depth->setSubpixel(false);

    xout->setStreamName("out");

    monoLeft->out.link(depth->left);
    monoRight->out.link(depth->right);
    depth->depth.link(pointcloud->inputDepth);
    pointcloud->outputPointCloud.link(xout->input);

    auto viewer = std::make_unique<pcl::visualization::PCLVisualizer>("Cloud Viewer");
    bool first = true;

    dai::Device device(pipeline);

    auto q = device.getOutputQueue("out", 8, false);
    long counter = 0;
    while(true) {
        std::cout << "Waiting for data" << std::endl;
        auto pclMsg = q->get<dai::PointCloudData>();
        std::cout << "Got data" << std::endl;
        if(!pclMsg) {
            std::cout << "No data" << std::endl;
            continue;
        }
        if(pclMsg->points.empty()) {
            std::cout << "Empty point cloud" << std::endl;
            continue;
        }
        std::cout << "Number of points: " << pclMsg->points.size() / 3 << std::endl;
        std::cout << "Min x: " << pclMsg->getMinX() << std::endl;
        std::cout << "Min y: " << pclMsg->getMinY() << std::endl;
        std::cout << "Min z: " << pclMsg->getMinZ() << std::endl;
        std::cout << "Max x: " << pclMsg->getMaxX() << std::endl;
        std::cout << "Max y: " << pclMsg->getMaxY() << std::endl;
        std::cout << "Max z: " << pclMsg->getMaxZ() << std::endl;

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = pclMsg->toPclData();
        if(first) {
            viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud");
            first = false;
        } else {
            viewer->updatePointCloud(cloud, "cloud");
        }

        viewer->spinOnce(200);

        if(viewer->wasStopped()) {
            break;
        }
    }

    return 0;
}
