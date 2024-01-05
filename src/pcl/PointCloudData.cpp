#include "depthai/pipeline/datatype/PointCloudData.hpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr dai::PointCloudData::toPclData() const {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	cloud->width = getWidth();
	cloud->height = getHeight();
	cloud->is_dense = true;
	cloud->points.resize(cloud->width * cloud->height);

	for (unsigned int i = 0; i < cloud->width * cloud->height; i++) {
		cloud->points[i].x = points[i].x;
		cloud->points[i].y = points[i].y;
		cloud->points[i].z = points[i].z;
	}

	return cloud;
}
