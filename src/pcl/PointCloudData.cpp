#include "depthai/pipeline/datatype/PointCloudData.hpp"

#include <execution>
pcl::PointCloud<pcl::PointXYZ>::Ptr dai::PointCloudData::getPclData() const {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    auto data = getData();
    cloud->width = getWidth();
    cloud->height = getHeight();
    cloud->is_dense = isSparse();

    auto* dataPtr = (Point3f*)data.data();
    auto size = data.size() / sizeof(Point3f);

    cloud->points.resize(size);

    for(unsigned int i = 0; i < size; i++) {
        cloud->points[i].x = dataPtr[i].x;
        cloud->points[i].y = dataPtr[i].y;
        cloud->points[i].z = dataPtr[i].z;
    }

    return cloud;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr dai::PointCloudData::getPclDataRGB() const {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    auto data = getData();
    cloud->width = getWidth();
    cloud->height = getHeight();
    cloud->is_dense = isSparse();

    auto* dataPtr = (Point3fRGB*)data.data();
    auto size = data.size() / sizeof(Point3fRGB);

    cloud->points.resize(size);

    std::for_each(std::execution::par, cloud->points.begin(), cloud->points.end(), [dataPtr, &cloud](pcl::PointXYZRGB& point) mutable {
        size_t i = &point - &cloud->points[0];
        point.x = dataPtr[i].x;
        point.y = dataPtr[i].y;
        point.z = dataPtr[i].z;
        point.r = dataPtr[i].r;
        point.g = dataPtr[i].g;
        point.b = dataPtr[i].b;
    });

    return cloud;
}

void dai::PointCloudData::setPclData(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    // Ensure cloud is valid
    if(!cloud) {
        throw std::invalid_argument("Input cloud is null");
    }
    auto size = cloud->points.size();
    std::vector<uint8_t> data(size * sizeof(Point3f));
    auto* dataPtr = reinterpret_cast<Point3f*>(data.data());
    setWidth(cloud->width);
    setHeight(cloud->height);
    setSparse(!cloud->is_dense);

    std::for_each(std::execution::par, cloud->points.begin(), cloud->points.end(), [dataPtr, &cloud](const pcl::PointXYZ& point) mutable {
        size_t i = &point - &cloud->points[0];
        dataPtr[i] = Point3f{point.x, point.y, point.z};
    });
    setData(data);
}

void dai::PointCloudData::setPclData(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud) {
    // Ensure cloud is valid
    if(!cloud) {
        throw std::invalid_argument("Input cloud is null");
    }

    auto size = cloud->points.size();
    std::vector<uint8_t> data(size * sizeof(Point3f));
    auto* dataPtr = reinterpret_cast<Point3f*>(data.data());
    setWidth(cloud->width);
    setHeight(cloud->height);
    setSparse(!cloud->is_dense);

    std::for_each(std::execution::par, cloud->points.begin(), cloud->points.end(), [dataPtr, &cloud](const pcl::PointXYZRGB& point) mutable {
        size_t i = &point - &cloud->points[0];
        dataPtr[i] = Point3f{point.x, point.y, point.z};
    });
    setData(data);
}
void dai::PointCloudData::setPclDataRGB(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud) {
    // Ensure cloud is valid
    if(!cloud) {
        throw std::invalid_argument("Input cloud is null");
    }

    auto size = cloud->points.size();
    std::vector<uint8_t> data(size * sizeof(Point3fRGB));
    auto* dataPtr = reinterpret_cast<Point3fRGB*>(data.data());
    setWidth(cloud->width);
    setHeight(cloud->height);
    setSparse(!cloud->is_dense);

    std::for_each(std::execution::par, cloud->points.begin(), cloud->points.end(), [dataPtr, &cloud](const pcl::PointXYZRGB& point) mutable {
        size_t i = &point - &cloud->points[0];
        dataPtr[i] = Point3fRGB{point.x, point.y, point.z, point.r, point.g, point.b};
    });
    color = true;
    setData(data);
}
