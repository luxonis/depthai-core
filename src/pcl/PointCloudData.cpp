#include "depthai/pipeline/datatype/PointCloudData.hpp"

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
