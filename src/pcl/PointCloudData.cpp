#include "depthai/pipeline/datatype/PointCloudData.hpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr dai::PointCloudData::toPclData() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    auto points = getPointsXYZ();

    cloud->width = getWidth();
    cloud->height = getHeight();
    cloud->is_dense = true;
    cloud->points.resize(cloud->width * cloud->height);

    for(unsigned int i = 0; i < cloud->width * cloud->height; i++) {
        cloud->points[i].x = points[i].x;
        cloud->points[i].y = points[i].y;
        cloud->points[i].z = points[i].z;
    }

    return cloud;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr dai::PointCloudData::toPclRGBData() {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    auto points = getPointsXYZRGB();

    cloud->width = getWidth();
    cloud->height = getHeight();
    cloud->is_dense = true;
    cloud->points.resize(cloud->width * cloud->height);

    for(unsigned int i = 0; i < cloud->width * cloud->height; i++) {
        cloud->points[i].x = points[i].x;
        cloud->points[i].y = points[i].y;
        cloud->points[i].z = points[i].z;
        cloud->points[i].r = points[i].r;
        cloud->points[i].g = points[i].g;
        cloud->points[i].b = points[i].b;
    }

    return cloud;
}
