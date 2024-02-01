#include "depthai/pipeline/datatype/PointCloudData.hpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr dai::PointCloudData::getPclData() const {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    cloud->width = getWidth();
    cloud->height = getHeight();
    cloud->is_dense = isSparse();
    cloud->points.resize(this->getData().size() / sizeof(Point3f));

    for(unsigned int i = 0; i < cloud->points.size(); i++) {
        cloud->points[i].x = points[i].x;
        cloud->points[i].y = points[i].y;
        cloud->points[i].z = points[i].z;
    }

    return cloud;
}
