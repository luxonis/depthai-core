#include "depthai/pipeline/datatype/DepthFrame.hpp"

#include <fstream>

namespace dai {

DepthFrame::DepthFrame() : ImgFrame() {}
DepthFrame::DepthFrame(long fd) : ImgFrame(fd) {}
DepthFrame::DepthFrame(size_t size) : ImgFrame(size) {}
DepthFrame::DepthFrame(long fd, size_t size) : ImgFrame(fd, size) {}

std::shared_ptr<PointCloudData> DepthFrame::getPointCloud() {
    auto intrinsics = transformation.getIntrinsicMatrix();
    auto fx = intrinsics[0][0];
    auto fy = intrinsics[1][1];
    auto cx = intrinsics[0][2];
    auto cy = intrinsics[1][2];

    std::shared_ptr<PointCloudData> pcl = std::make_shared<PointCloudData>();
    auto width = getWidth();
    auto height = getHeight();
    pcl->setWidth(width);
    pcl->setHeight(height);
    auto data = getData().data();
    std::vector<Point3f> points;
    points.reserve(width * height);
    float scale = 1.0f;
    for(uint row = 0; row < height; row++) {
        for(uint col = 0; col < width; col++) {
            size_t i = row * width + col;
            uint16_t depth = *(reinterpret_cast<const uint16_t*>(data + i * sizeof(uint16_t)));
            float z = depth / getScale();
            float x = (col - cx) * z / fx;
            float y = (row - cy) * z / fy;
            points.push_back({x, y, z});
        }
    }
    return pcl;
}

void DepthFrame::save(const std::string& path) {
    // Example PCD file
    // # .PCD v.7 - Point Cloud Data file format
    // VERSION .7
    // FIELDS x y z rgb
    // SIZE 4 4 4 4
    // TYPE F F F F
    // COUNT 1 1 1 1
    // WIDTH 213
    // HEIGHT 1
    // VIEWPOINT 0 0 0 1 0 0 0
    // POINTS 213
    // DATA ascii
    // 0.93773 0.33763 0 4.2108e+06
    // 0.90805 0.35641 0 4.2108e+06

    auto pcl = getPointCloud();

    std::ofstream file(path);
    file << "# .PCD v.7 - Point Cloud Data file format\n";
    file << "VERSION .7\n";
    file << "FIELDS x y z\n";
    file << "SIZE 4 4 4\n";
    file << "TYPE F F F\n";
    file << "COUNT 1 1 1\n";
    file << "WIDTH " << pcl->getWidth() << "\n";
    file << "HEIGHT " << pcl->getHeight() << "\n";
    file << "VIEWPOINT 0 0 0 1 0 0 0\n";
    file << "POINTS " << pcl->getWidth() * pcl->getHeight() << "\n";
    file << "DATA ascii\n";
    for(auto& p : pcl->getPoints()) {
        file << p.x << " " << p.y << " " << p.z << "\n";
    }
    file.close();
}

float DepthFrame::getMedian() {
    if(median != 0) {
        return median;
    }
    auto data = getData().data();
    // copy data to vector
    std::vector<uint16_t> depthData(getWidth() * getHeight());
    std::copy(reinterpret_cast<const uint16_t*>(data), reinterpret_cast<const uint16_t*>(data + getWidth() * getHeight()), depthData.begin());
    // sort
    std::sort(depthData.begin(), depthData.end());
    // calculate median
    float med = 0;
    if(depthData.size() % 2 == 0) {
        med = (depthData[depthData.size() / 2 - 1] + depthData[depthData.size() / 2]) / 2.0f;
    } else {
        med = depthData[depthData.size() / 2];
    }
    float z = med / getScale();
    median = z;
    return median;
}

float DepthFrame::getMin() {
    if(min != 0) {
        return min;
    }
    auto data = getData().data();
    // iterate and find min
    uint16_t min = *std::min_element(reinterpret_cast<const uint16_t*>(data), reinterpret_cast<const uint16_t*>(data + getData().size()));
    float z = min / getScale();
    return z;
}

float DepthFrame::getMax() {
    auto data = getData().data();
    // iterate and find max
    uint16_t max = *std::max_element(reinterpret_cast<const uint16_t*>(data), reinterpret_cast<const uint16_t*>(data + getData().size()));
    float z = max / getScale();
    return z;
}

DepthSource DepthFrame::getSource() const {
    return source;
}

CameraBoardSocket DepthFrame::getLeftCamera() const {
    return leftCamera;
}

CameraBoardSocket DepthFrame::getRightCamera() const {
    return rightCamera;
}

float DepthFrame::getScale() {
    if(source == DepthSource::Stereo) {
        auto depthUnit = std::get<StereoDepthConfig>(config).algorithmControl.depthUnit;
        if(depthUnit == StereoDepthConfig::AlgorithmControl::DepthUnit::MILLIMETER) {
            scale = 1.0f;
        } else if(depthUnit == StereoDepthConfig::AlgorithmControl::DepthUnit::CENTIMETER) {
            scale = 10.0f;
        } else if(depthUnit == StereoDepthConfig::AlgorithmControl::DepthUnit::METER) {
            scale = 1000.0f;
        } else {
            scale = 1.0f;
        }
    } else {
        scale = 1.0f;
    }
    return scale;
}
}  // namespace dai
