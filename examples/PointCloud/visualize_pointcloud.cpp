#include "depthai/depthai.hpp"
#include <iostream>

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

	dai::Device device(pipeline);

    auto q = device.getOutputQueue("out", 8, false);
    while(true) {
        std::cout << "Waiting for data" << std::endl;
        const auto pcl = q->get<dai::PointCloudData>();
        std::cout << "Got data" << std::endl;
        if(!pcl) {
            std::cout << "No data" << std::endl;
            continue;
        }
        std::cout << "Number of points: " << pcl->points.size() / 3 << std::endl;
        std::cout << "Min x: " << pcl->getMinX() << std::endl;
        std::cout << "Min y: " << pcl->getMinY() << std::endl;
        std::cout << "Min z: " << pcl->getMinZ() << std::endl;
        std::cout << "Max x: " << pcl->getMaxX() << std::endl;
        std::cout << "Max y: " << pcl->getMaxY() << std::endl;
        std::cout << "Max z: " << pcl->getMaxZ() << std::endl;
        pcl->visualizePcl();
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        break;
    }

    return 0;
}
