#include "depthai/pipeline/datatype/PointCloudData.hpp"

#include "depthai/schemas/PointCloudData.pb.h"

#include <algorithm>
pcl::PointCloud<pcl::PointXYZ>::Ptr dai::PointCloudData::getPclData() const {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    auto data = getData();
    cloud->width = getWidth();
    cloud->height = getHeight();
    cloud->is_dense = isSparse();
    if(isColor()) {
        auto* dataPtr = (Point3fRGB*)data.data();
        auto size = data.size() / sizeof(Point3fRGB);

        cloud->points.resize(size);

        std::for_each(cloud->points.begin(), cloud->points.end(), [dataPtr, &cloud](pcl::PointXYZ& point) mutable {
            size_t i = &point - &cloud->points[0];
            point.x = dataPtr[i].x;
            point.y = dataPtr[i].y;
            point.z = dataPtr[i].z;
        });
        return cloud;
    }
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
    if(!isColor()) {
        throw std::runtime_error("PointCloudData does not contain color data");
    }
    auto data = getData();
    cloud->width = getWidth();
    cloud->height = getHeight();
    cloud->is_dense = isSparse();

    auto* dataPtr = (Point3fRGB*)data.data();
    auto size = data.size() / sizeof(Point3fRGB);

    cloud->points.resize(size);

    std::for_each(cloud->points.begin(), cloud->points.end(), [dataPtr, &cloud](pcl::PointXYZRGB& point) mutable {
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

    std::for_each(cloud->points.begin(), cloud->points.end(), [dataPtr, &cloud](const pcl::PointXYZ& point) mutable {
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

    std::for_each(cloud->points.begin(), cloud->points.end(), [dataPtr, &cloud](const pcl::PointXYZRGB& point) mutable {
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

    std::for_each(cloud->points.begin(), cloud->points.end(), [dataPtr, &cloud](const pcl::PointXYZRGB& point) mutable {
        size_t i = &point - &cloud->points[0];
        dataPtr[i] = Point3fRGB{point.x, point.y, point.z, point.r, point.g, point.b};
    });
    color = true;
    setData(data);
}

std::unique_ptr<google::protobuf::Message> dai::PointCloudData::getProtoMessage() const {
    auto pointCloudData = std::make_unique<proto::PointCloudData>();

    auto timestamp = pointCloudData->mutable_ts();
    timestamp->set_sec(ts.sec);
    timestamp->set_nsec(ts.nsec);

    auto timestampDevice = pointCloudData->mutable_tsdevice();
    timestampDevice->set_sec(tsDevice.sec);
    timestampDevice->set_nsec(tsDevice.nsec);

    pointCloudData->set_sequencenum(sequenceNum);
    pointCloudData->set_width(width);
    pointCloudData->set_height(height);
    pointCloudData->set_instancenum(instanceNum);
    pointCloudData->set_minx(minx);
    pointCloudData->set_miny(miny);
    pointCloudData->set_minz(minz);
    pointCloudData->set_maxx(maxx);
    pointCloudData->set_maxy(maxy);
    pointCloudData->set_maxz(maxz);
    pointCloudData->set_sparse(sparse);
    pointCloudData->set_color(color);

    pointCloudData->set_data(data->getData().data(), data->getSize());

    return pointCloudData;
}