#pragma once

#include <string>

#include "depthai/pipeline/datatype/DatatypeEnum.hpp"
#include "depthai/schemas/EncodedFrame.pb.h"
#include "depthai/schemas/IMUData.pb.h"
#include "depthai/schemas/ImageAnnotations.pb.h"
#include "depthai/schemas/ImgDetections.pb.h"
#include "depthai/schemas/ImgFrame.pb.h"
#include "depthai/schemas/PointCloudData.pb.h"
#include "depthai/schemas/SpatialImgDetections.pb.h"

namespace dai {
namespace utility {

inline DatatypeEnum schemaNameToDatatype(const std::string& schemaName) {
    if(schemaName == proto::encoded_frame::EncodedFrame::descriptor()->full_name()) {
        return DatatypeEnum::EncodedFrame;
    } else if(schemaName == proto::imu_data::IMUData::descriptor()->full_name()) {
        return DatatypeEnum::IMUData;
    } else if(schemaName == proto::image_annotations::ImageAnnotations::descriptor()->full_name()) {
        return DatatypeEnum::ImageAnnotations;
    } else if(schemaName == proto::img_detections::ImgDetections::descriptor()->full_name()) {
        return DatatypeEnum::ImgDetections;
    } else if(schemaName == proto::img_frame::ImgFrame::descriptor()->full_name()) {
        return DatatypeEnum::ImgFrame;
    } else if(schemaName == proto::point_cloud_data::PointCloudData::descriptor()->full_name()) {
        return DatatypeEnum::PointCloudData;
    } else if(schemaName == proto::spatial_img_detections::SpatialImgDetections::descriptor()->full_name()) {
        return DatatypeEnum::SpatialImgDetections;
    } else {
        throw std::runtime_error("Unknown schema name: " + schemaName);
    }
}

}  // namespace utility
}  // namespace dai
