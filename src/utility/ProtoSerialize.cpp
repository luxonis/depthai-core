#include "ProtoSerialize.hpp"

#include <google/protobuf/descriptor.pb.h>
#include <google/protobuf/message.h>
#include <google/protobuf/util/time_util.h>

#include <cstdint>
#include <optional>
#include <queue>

#include "depthai/schemas/PointCloudData.pb.h"
#include "depthai/schemas/SegmentationMask.pb.h"
#include "depthai/schemas/common.pb.h"
#include "pipeline/datatype/DatatypeEnum.hpp"
#include "pipeline/datatype/ImgDetections.hpp"
#include "pipeline/datatype/SegmentationMask.hpp"

namespace dai {
namespace utility {
// // Writes the FileDescriptor of this descriptor and all transitive dependencies
// // to a string, for use as a channel schema.
static std::string serializeFdSet(const google::protobuf::Descriptor* toplevelDescriptor) {
    google::protobuf::FileDescriptorSet fdSet;
    std::queue<const google::protobuf::FileDescriptor*> toAdd;
    toAdd.push(toplevelDescriptor->file());
    std::unordered_set<std::string> seenDependencies;
    while(!toAdd.empty()) {
        const google::protobuf::FileDescriptor* next = toAdd.front();
        toAdd.pop();
        next->CopyTo(fdSet.add_file());
        for(int i = 0; i < next->dependency_count(); ++i) {
            const auto& dep = next->dependency(i);
            if(seenDependencies.find(dep->name()) == seenDependencies.end()) {
                seenDependencies.insert(dep->name());
                toAdd.push(dep);
            }
        }
    }
    return fdSet.SerializeAsString();
}

std::vector<std::uint8_t> serializeProto(std::unique_ptr<google::protobuf::Message> protoMessage) {
    std::size_t nbytes = protoMessage->ByteSizeLong();
    std::vector<std::uint8_t> buffer(nbytes);

    // The test is necessary becaue v.data could be NULL if nbytes is 0
    if(nbytes > 0) {
        protoMessage->SerializeToArray(buffer.data(), nbytes);
    }

    return buffer;
}

ProtoSerializable::SchemaPair serializeSchema(std::unique_ptr<google::protobuf::Message> protoMessage) {
    const auto* descriptor = protoMessage->GetDescriptor();
    if(descriptor == nullptr) {
        throw std::runtime_error("Failed to get protobuf descriptor");
    }
    ProtoSerializable::SchemaPair returnPair;
    returnPair.schemaName = descriptor->full_name();
    returnPair.schema = serializeFdSet(descriptor);
    return returnPair;
}

void serializeImgTransformation(proto::common::ImgTransformation* imgTransformation, const ImgTransformation& transformation) {
    const auto [width, height] = transformation.getSize();
    const auto [srcWidth, srcHeight] = transformation.getSourceSize();
    imgTransformation->set_width(width);
    imgTransformation->set_height(height);
    imgTransformation->set_srcwidth(srcWidth);
    imgTransformation->set_srcheight(srcHeight);

    proto::common::TransformationMatrix* transformationMatrix = imgTransformation->mutable_transformationmatrix();
    for(const auto& array : transformation.getMatrix()) {
        proto::common::FloatArray* floatArray = transformationMatrix->add_arrays();
        for(const auto& value : array) {
            floatArray->add_values(value);
        }
    }
    proto::common::TransformationMatrix* sourceIntrinsicMatrix = imgTransformation->mutable_sourceintrinsicmatrix();
    for(const auto& array : transformation.getSourceIntrinsicMatrix()) {
        proto::common::FloatArray* floatArray = sourceIntrinsicMatrix->add_arrays();
        for(const auto& value : array) {
            floatArray->add_values(value);
        }
    }

    imgTransformation->set_distortionmodel(static_cast<proto::common::CameraModel>(transformation.getDistortionModel()));
    proto::common::FloatArray* distortionCoefficients = imgTransformation->mutable_distortioncoefficients();
    for(const auto& value : transformation.getDistortionCoefficients()) {
        distortionCoefficients->add_values(value);
    }
}
ImgTransformation deserializeImgTransformation(const proto::common::ImgTransformation& imgTransformation) {
    std::array<std::array<float, 3>, 3> transformationMatrix;
    std::array<std::array<float, 3>, 3> sourceIntrinsicMatrix;
    std::vector<float> distortionCoefficients;
    distortionCoefficients.reserve(imgTransformation.distortioncoefficients().values_size());
    for(auto i = 0U; i < 3; ++i)
        for(auto j = 0U; j < 3; ++j) transformationMatrix[i][j] = imgTransformation.transformationmatrix().arrays(i).values(j);
    for(auto i = 0U; i < 3; ++i)
        for(auto j = 0U; j < 3; ++j) sourceIntrinsicMatrix[i][j] = imgTransformation.sourceintrinsicmatrix().arrays(i).values(j);
    for(auto i = 0; i < imgTransformation.distortioncoefficients().values_size(); ++i)
        distortionCoefficients.push_back(imgTransformation.distortioncoefficients().values(i));
    ImgTransformation transformation;
    transformation = ImgTransformation(imgTransformation.srcwidth(),
                                       imgTransformation.srcheight(),
                                       sourceIntrinsicMatrix,
                                       static_cast<CameraModel>(imgTransformation.distortionmodel()),
                                       distortionCoefficients);
    if(transformation.isValid()) {
        transformation.addTransformation(transformationMatrix);
        transformation.addCrop(0, 0, imgTransformation.width(), imgTransformation.height());
    }
    return transformation;
}

DatatypeEnum schemaNameToDatatype(const std::string& schemaName) {
    if(schemaName == proto::encoded_frame::EncodedFrame::descriptor()->full_name()) {
        return DatatypeEnum::EncodedFrame;
    } else if(schemaName == proto::imu_data::IMUData::descriptor()->full_name()) {
        return DatatypeEnum::IMUData;
    } else if(schemaName == proto::image_annotations::ImageAnnotations::descriptor()->full_name()) {
        return DatatypeEnum::ImgAnnotations;
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

bool deserializationSupported(DatatypeEnum datatype) {
    switch(datatype) {
        case DatatypeEnum::ImgFrame:
        case DatatypeEnum::EncodedFrame:
        case DatatypeEnum::IMUData:
        case DatatypeEnum::PointCloudData:
            return true;
        case DatatypeEnum::ADatatype:
        case DatatypeEnum::Buffer:
        case DatatypeEnum::NNData:
        case DatatypeEnum::ImageManipConfig:
        case DatatypeEnum::CameraControl:
        case DatatypeEnum::ImgDetections:
        case DatatypeEnum::SegmentationMask:
        case DatatypeEnum::SpatialImgDetections:
        case DatatypeEnum::SystemInformation:
        case DatatypeEnum::SystemInformationS3:
        case DatatypeEnum::SpatialLocationCalculatorConfig:
        case DatatypeEnum::SegmentationParserConfig:
        case DatatypeEnum::SpatialLocationCalculatorData:
        case DatatypeEnum::EdgeDetectorConfig:
        case DatatypeEnum::AprilTagConfig:
        case DatatypeEnum::AprilTags:
        case DatatypeEnum::Tracklets:
        case DatatypeEnum::StereoDepthConfig:
        case DatatypeEnum::NeuralDepthConfig:
        case DatatypeEnum::FeatureTrackerConfig:
        case DatatypeEnum::ThermalConfig:
        case DatatypeEnum::ToFConfig:
        case DatatypeEnum::TrackedFeatures:
        case DatatypeEnum::BenchmarkReport:
        case DatatypeEnum::MessageGroup:
        case DatatypeEnum::TransformData:
        case DatatypeEnum::PointCloudConfig:
        case DatatypeEnum::ImageAlignConfig:
        case DatatypeEnum::ImgAnnotations:
        case DatatypeEnum::ImageFiltersConfig:
        case DatatypeEnum::ToFDepthConfidenceFilterConfig:
        case DatatypeEnum::RGBDData:
        case DatatypeEnum::ObjectTrackerConfig:
        case DatatypeEnum::DynamicCalibrationControl:
        case DatatypeEnum::DynamicCalibrationResult:
        case DatatypeEnum::CalibrationQuality:
        case DatatypeEnum::CoverageData:
            return false;
    }
    return false;
}

template <>
std::unique_ptr<google::protobuf::Message> getProtoMessage(const ImgAnnotations* message, bool) {
    auto imageAnnotations = std::make_unique<proto::image_annotations::ImageAnnotations>();

    imageAnnotations->set_sequencenum(message->sequenceNum);
    proto::common::Timestamp* ts = imageAnnotations->mutable_ts();
    ts->set_sec(message->ts.sec);
    ts->set_nsec(message->ts.nsec);
    proto::common::Timestamp* tsDevice = imageAnnotations->mutable_tsdevice();
    tsDevice->set_sec(message->tsDevice.sec);
    tsDevice->set_nsec(message->tsDevice.nsec);

    for(const auto& annotation : message->annotations) {
        proto::image_annotations::ImageAnnotation* imageAnnotation = imageAnnotations->add_annotations();
        for(const auto& circle : annotation.circles) {
            proto::image_annotations::CircleAnnotation* circleAnnotation = imageAnnotation->add_circles();
            circleAnnotation->mutable_position()->set_x(circle.position.x);
            circleAnnotation->mutable_position()->set_y(circle.position.y);
            circleAnnotation->set_diameter(circle.diameter);
            circleAnnotation->set_thickness(circle.thickness);
            circleAnnotation->mutable_fillcolor()->set_r(circle.fillColor.r);
            circleAnnotation->mutable_fillcolor()->set_g(circle.fillColor.g);
            circleAnnotation->mutable_fillcolor()->set_b(circle.fillColor.b);
            circleAnnotation->mutable_fillcolor()->set_a(circle.fillColor.a);
            circleAnnotation->mutable_outlinecolor()->set_r(circle.outlineColor.r);
            circleAnnotation->mutable_outlinecolor()->set_g(circle.outlineColor.g);
            circleAnnotation->mutable_outlinecolor()->set_b(circle.outlineColor.b);
            circleAnnotation->mutable_outlinecolor()->set_a(circle.outlineColor.a);
        }
        for(const auto& points : annotation.points) {
            proto::image_annotations::PointsAnnotation* pointsAnnotation = imageAnnotation->add_points();
            PointsAnnotationType type = points.type;
            pointsAnnotation->set_type(static_cast<proto::image_annotations::PointsAnnotationType>(type));
            for(const auto& point : points.points) {
                proto::common::Point2f* protoPoint = pointsAnnotation->add_points();
                protoPoint->set_x(point.x);
                protoPoint->set_y(point.y);
            }
            pointsAnnotation->mutable_outlinecolor()->set_r(points.outlineColor.r);
            pointsAnnotation->mutable_outlinecolor()->set_g(points.outlineColor.g);
            pointsAnnotation->mutable_outlinecolor()->set_b(points.outlineColor.b);
            pointsAnnotation->mutable_outlinecolor()->set_a(points.outlineColor.a);
            for(const auto& color : points.outlineColors) {
                proto::common::Color* protoColor = pointsAnnotation->add_outlinecolors();
                protoColor->set_r(color.r);
                protoColor->set_g(color.g);
                protoColor->set_b(color.b);
                protoColor->set_a(color.a);
            }
            pointsAnnotation->mutable_fillcolor()->set_r(points.fillColor.r);
            pointsAnnotation->mutable_fillcolor()->set_g(points.fillColor.g);
            pointsAnnotation->mutable_fillcolor()->set_b(points.fillColor.b);
            pointsAnnotation->mutable_fillcolor()->set_a(points.fillColor.a);
            pointsAnnotation->set_thickness(points.thickness);
        }
        for(const auto& text : annotation.texts) {
            proto::image_annotations::TextAnnotation* textAnnotation = imageAnnotation->add_texts();
            textAnnotation->mutable_position()->set_x(text.position.x);
            textAnnotation->mutable_position()->set_y(text.position.y);
            textAnnotation->set_text(text.text);
            textAnnotation->set_fontsize(text.fontSize);
            textAnnotation->mutable_textcolor()->set_r(text.textColor.r);
            textAnnotation->mutable_textcolor()->set_g(text.textColor.g);
            textAnnotation->mutable_textcolor()->set_b(text.textColor.b);
            textAnnotation->mutable_textcolor()->set_a(text.textColor.a);
            textAnnotation->mutable_backgroundcolor()->set_r(text.backgroundColor.r);
            textAnnotation->mutable_backgroundcolor()->set_g(text.backgroundColor.g);
            textAnnotation->mutable_backgroundcolor()->set_b(text.backgroundColor.b);
            textAnnotation->mutable_backgroundcolor()->set_a(text.backgroundColor.a);
        }
    }
    return imageAnnotations;
}
template <>
std::unique_ptr<google::protobuf::Message> getProtoMessage(const SpatialImgDetections* message, bool metadataOnly) {
    // create and populate SpatialImgDetections protobuf message
    auto spatialImgDetections = std::make_unique<proto::spatial_img_detections::SpatialImgDetections>();
    spatialImgDetections->set_sequencenum(message->sequenceNum);

    proto::common::Timestamp* ts = spatialImgDetections->mutable_ts();
    ts->set_sec(message->ts.sec);
    ts->set_nsec(message->ts.nsec);

    proto::common::Timestamp* tsDevice = spatialImgDetections->mutable_tsdevice();
    tsDevice->set_sec(message->tsDevice.sec);
    tsDevice->set_nsec(message->tsDevice.nsec);

    for(const auto& detection : message->detections) {
        proto::spatial_img_detections::SpatialImgDetection* spatialImgDetection = spatialImgDetections->add_detections();

        // populate SpatialImgDetection.ImgDetection from struct inheritance

        spatialImgDetection->set_label(detection.label);
        spatialImgDetection->set_labelname(detection.labelName);
        spatialImgDetection->set_confidence(detection.confidence);
        spatialImgDetection->set_xmin(detection.xmin);
        spatialImgDetection->set_ymin(detection.ymin);
        spatialImgDetection->set_xmax(detection.xmax);
        spatialImgDetection->set_ymax(detection.ymax);

        if(detection.boundingBox.has_value() || !(detection.xmin == 0.f && detection.xmax == 0.f && detection.ymin == 0.f && detection.ymax == 0.f)) {
            const auto bbox = detection.boundingBox.has_value() ? detection.boundingBox.value() : detection.getBoundingBox();
            proto::common::RotatedRect* bboxProto = spatialImgDetection->mutable_boundingbox();
            proto::common::Point2f* center = bboxProto->mutable_center();
            center->set_x(bbox.center.x);
            center->set_y(bbox.center.y);
            proto::common::Size2f* size = bboxProto->mutable_size();
            size->set_width(bbox.size.width);
            size->set_height(bbox.size.height);
            bboxProto->set_angle(bbox.angle);
        }

        // populate SpatialImgDetection.Point3f
        proto::spatial_img_detections::Point3f* spatialCoordinates = spatialImgDetection->mutable_spatialcoordinates();
        spatialCoordinates->set_x(detection.spatialCoordinates.x);
        spatialCoordinates->set_y(detection.spatialCoordinates.y);
        spatialCoordinates->set_z(detection.spatialCoordinates.z);

        // populate SpatialImgDetection.SpatialLocationCalculatorConfigData
        proto::spatial_img_detections::SpatialLocationCalculatorConfigData* boundingBoxMapping = spatialImgDetection->mutable_boundingboxmapping();

        // populate SpatialImgDetection.SpatialLocationCalculatorConfigData.Rect
        proto::spatial_img_detections::Rect* roi = boundingBoxMapping->mutable_roi();
        roi->set_x(detection.boundingBoxMapping.roi.x);
        roi->set_y(detection.boundingBoxMapping.roi.y);
        roi->set_width(detection.boundingBoxMapping.roi.width);
        roi->set_height(detection.boundingBoxMapping.roi.height);

        // populate SpatialImgDetection.SpatialLocationCalculatorConfigData.SpatialLocationCalculatorConfigThresholds
        proto::spatial_img_detections::SpatialLocationCalculatorConfigThresholds* depthTresholds = boundingBoxMapping->mutable_depththresholds();
        depthTresholds->set_lowerthreshold(detection.boundingBoxMapping.depthThresholds.lowerThreshold);
        depthTresholds->set_upperthreshold(detection.boundingBoxMapping.depthThresholds.upperThreshold);

        // populate SpatialImgDetection.SpatialLocationCalculatorConfigData.SpatialLocationCalculatorAlgorithm
        boundingBoxMapping->set_calculationalgorithm(
            static_cast<proto::spatial_img_detections::SpatialLocationCalculatorAlgorithm>(detection.boundingBoxMapping.calculationAlgorithm));

        // populate SpatialImgDetection.SpatialLocationCalculatorConfigData.stepSize
        boundingBoxMapping->set_stepsize(detection.boundingBoxMapping.stepSize);

        if(detection.keypoints.has_value()) {
            const auto& keypointsList = detection.keypoints.value();
            const auto keypointsVec = keypointsList.getKeypoints();
            const auto edgesVec = keypointsList.getEdges();
            proto::common::SpatialKeypointsList* protoKeypoints = spatialImgDetection->mutable_keypoints();
            for(const auto& keypoint : keypointsVec) {
                auto* protoKeypoint = protoKeypoints->add_keypoints();
                proto::common::Point3f* coords = protoKeypoint->mutable_imagecoordinates();
                coords->set_x(keypoint.imageCoordinates.x);
                coords->set_y(keypoint.imageCoordinates.y);
                coords->set_z(keypoint.imageCoordinates.z);
                protoKeypoint->set_confidence(keypoint.confidence);
                protoKeypoint->set_label(keypoint.label);

                proto::common::Point3f* spatialCoords = protoKeypoint->mutable_spatialcoordinates();
                spatialCoords->set_x(keypoint.spatialCoordinates.x);
                spatialCoords->set_y(keypoint.spatialCoordinates.y);
                spatialCoords->set_z(keypoint.spatialCoordinates.z);
            }
            for(const auto& edge : edgesVec) {
                auto* protoEdge = protoKeypoints->add_edges();
                protoEdge->set_src(edge[0]);
                protoEdge->set_dst(edge[1]);
            }
        }
    }

    proto::common::ImgTransformation* imgTransformation = spatialImgDetections->mutable_transformation();
    if(message->transformation.has_value()) {
        utility::serializeImgTransformation(imgTransformation, message->transformation.value());
    }

    spatialImgDetections->set_segmentationmaskwidth(static_cast<std::int64_t>(message->getSegmentationMaskWidth()));
    spatialImgDetections->set_segmentationmaskheight(static_cast<std::int64_t>(message->getSegmentationMaskHeight()));

    if(!metadataOnly) {
        std::optional<std::vector<std::uint8_t>> segMaskData = message->getMaskData();
        if(segMaskData) {
            spatialImgDetections->set_data((*segMaskData).data(), (*segMaskData).size());
        }
    }
    return spatialImgDetections;
}
template <>
std::unique_ptr<google::protobuf::Message> getProtoMessage(const IMUData* message, bool) {
    // create and populate ImgFrame protobuf message
    auto imuData = std::make_unique<proto::imu_data::IMUData>();
    auto imuPackets = imuData->mutable_packets();
    imuPackets->Reserve(message->packets.size());
    for(auto packet : message->packets) {
        auto imuPacket = imuPackets->Add();
        auto imuAccelerometer = imuPacket->mutable_accelerometer();
        imuAccelerometer->mutable_vec()->set_x(packet.acceleroMeter.x);
        imuAccelerometer->mutable_vec()->set_y(packet.acceleroMeter.y);
        imuAccelerometer->mutable_vec()->set_z(packet.acceleroMeter.z);
        imuAccelerometer->mutable_report()->set_accuracy(static_cast<dai::proto::imu_data::Accuracy>(packet.acceleroMeter.accuracy));
        imuAccelerometer->mutable_report()->set_sequence(packet.acceleroMeter.sequence);
        imuAccelerometer->mutable_report()->mutable_ts()->set_sec(packet.acceleroMeter.timestamp.sec);
        imuAccelerometer->mutable_report()->mutable_ts()->set_nsec(packet.acceleroMeter.timestamp.nsec);
        imuAccelerometer->mutable_report()->mutable_tsdevice()->set_sec(packet.acceleroMeter.tsDevice.sec);
        imuAccelerometer->mutable_report()->mutable_tsdevice()->set_nsec(packet.acceleroMeter.tsDevice.nsec);

        auto imuGyroscope = imuPacket->mutable_gyroscope();
        imuGyroscope->mutable_vec()->set_x(packet.gyroscope.x);
        imuGyroscope->mutable_vec()->set_y(packet.gyroscope.y);
        imuGyroscope->mutable_vec()->set_z(packet.gyroscope.z);
        imuGyroscope->mutable_report()->set_accuracy(static_cast<dai::proto::imu_data::Accuracy>(packet.gyroscope.accuracy));
        imuGyroscope->mutable_report()->set_sequence(packet.gyroscope.sequence);
        imuGyroscope->mutable_report()->mutable_ts()->set_sec(packet.gyroscope.timestamp.sec);
        imuGyroscope->mutable_report()->mutable_ts()->set_nsec(packet.gyroscope.timestamp.nsec);
        imuGyroscope->mutable_report()->mutable_tsdevice()->set_sec(packet.gyroscope.tsDevice.sec);
        imuGyroscope->mutable_report()->mutable_tsdevice()->set_nsec(packet.gyroscope.tsDevice.nsec);

        auto imuMagnetometer = imuPacket->mutable_magnetometer();
        imuMagnetometer->mutable_vec()->set_x(packet.magneticField.x);
        imuMagnetometer->mutable_vec()->set_y(packet.magneticField.y);
        imuMagnetometer->mutable_vec()->set_z(packet.magneticField.z);
        imuMagnetometer->mutable_report()->set_accuracy(static_cast<dai::proto::imu_data::Accuracy>(packet.magneticField.accuracy));
        imuMagnetometer->mutable_report()->set_sequence(packet.magneticField.sequence);
        imuMagnetometer->mutable_report()->mutable_ts()->set_sec(packet.magneticField.timestamp.sec);
        imuMagnetometer->mutable_report()->mutable_ts()->set_nsec(packet.magneticField.timestamp.nsec);
        imuMagnetometer->mutable_report()->mutable_tsdevice()->set_sec(packet.magneticField.tsDevice.sec);
        imuMagnetometer->mutable_report()->mutable_tsdevice()->set_nsec(packet.magneticField.tsDevice.nsec);

        auto imuRotationVector = imuPacket->mutable_rotationvector();
        imuRotationVector->mutable_quat()->set_x(packet.rotationVector.i);
        imuRotationVector->mutable_quat()->set_y(packet.rotationVector.j);
        imuRotationVector->mutable_quat()->set_z(packet.rotationVector.k);
        imuRotationVector->mutable_quat()->set_w(packet.rotationVector.real);
        imuRotationVector->mutable_report()->set_accuracy(static_cast<dai::proto::imu_data::Accuracy>(packet.rotationVector.accuracy));
        imuRotationVector->mutable_report()->set_sequence(packet.rotationVector.sequence);
        imuRotationVector->mutable_report()->mutable_ts()->set_sec(packet.rotationVector.timestamp.sec);
        imuRotationVector->mutable_report()->mutable_ts()->set_nsec(packet.rotationVector.timestamp.nsec);
        imuRotationVector->mutable_report()->mutable_tsdevice()->set_sec(packet.rotationVector.tsDevice.sec);
        imuRotationVector->mutable_report()->mutable_tsdevice()->set_nsec(packet.rotationVector.tsDevice.nsec);
    }

    // Set timestamps
    proto::common::Timestamp* ts = imuData->mutable_ts();
    ts->set_sec(message->ts.sec);
    ts->set_nsec(message->ts.nsec);
    proto::common::Timestamp* tsDevice = imuData->mutable_tsdevice();
    tsDevice->set_sec(message->tsDevice.sec);
    tsDevice->set_nsec(message->tsDevice.nsec);
    imuData->set_sequencenum(message->sequenceNum);

    return imuData;
}
template <>
std::unique_ptr<google::protobuf::Message> getProtoMessage(const ImgDetections* message, bool metadataOnly) {
    auto imgDetections = std::make_unique<proto::img_detections::ImgDetections>();

    imgDetections->set_sequencenum(message->sequenceNum);
    proto::common::Timestamp* ts = imgDetections->mutable_ts();
    ts->set_sec(message->ts.sec);
    ts->set_nsec(message->ts.nsec);
    proto::common::Timestamp* tsDevice = imgDetections->mutable_tsdevice();
    tsDevice->set_sec(message->tsDevice.sec);
    tsDevice->set_nsec(message->tsDevice.nsec);

    for(const auto& detection : message->detections) {
        proto::img_detections::ImgDetection* imgDetection = imgDetections->add_detections();
        imgDetection->set_label(detection.label);
        imgDetection->set_labelname(detection.labelName);
        imgDetection->set_confidence(detection.confidence);
        imgDetection->set_xmin(detection.xmin);
        imgDetection->set_ymin(detection.ymin);
        imgDetection->set_xmax(detection.xmax);
        imgDetection->set_ymax(detection.ymax);

        if(detection.boundingBox.has_value() || !(detection.xmin == 0.f && detection.xmax == 0.f && detection.ymin == 0.f && detection.ymax == 0.f)) {
            const auto bbox = detection.boundingBox.has_value() ? detection.boundingBox.value() : detection.getBoundingBox();
            proto::common::RotatedRect* bboxProto = imgDetection->mutable_boundingbox();
            proto::common::Point2f* center = bboxProto->mutable_center();
            center->set_x(bbox.center.x);
            center->set_y(bbox.center.y);
            proto::common::Size2f* size = bboxProto->mutable_size();
            size->set_width(bbox.size.width);
            size->set_height(bbox.size.height);
            bboxProto->set_angle(bbox.angle);
        }

        if(detection.keypoints.has_value()) {
            const auto& keypointsList = detection.keypoints.value();
            const auto keypointsVec = keypointsList.getKeypoints();
            const auto edgesVec = keypointsList.getEdges();
            proto::common::KeypointsList* protoKeypoints = imgDetection->mutable_keypoints();
            for(const auto& keypoint : keypointsVec) {
                auto* protoKeypoint = protoKeypoints->add_keypoints();
                proto::common::Point3f* coords = protoKeypoint->mutable_imagecoordinates();
                coords->set_x(keypoint.imageCoordinates.x);
                coords->set_y(keypoint.imageCoordinates.y);
                coords->set_z(keypoint.imageCoordinates.z);
                protoKeypoint->set_confidence(keypoint.confidence);
                protoKeypoint->set_label(keypoint.label);
            }
            for(const auto& edge : edgesVec) {
                auto* protoEdge = protoKeypoints->add_edges();
                protoEdge->set_src(edge[0]);
                protoEdge->set_dst(edge[1]);
            }
        }
    }

    imgDetections->set_segmentationmaskwidth(static_cast<std::int64_t>(message->getSegmentationMaskWidth()));
    imgDetections->set_segmentationmaskheight(static_cast<std::int64_t>(message->getSegmentationMaskHeight()));

    proto::common::ImgTransformation* imgTransformation = imgDetections->mutable_transformation();
    if(message->transformation.has_value()) {
        utility::serializeImgTransformation(imgTransformation, message->transformation.value());
    }

    if(!metadataOnly) {
        std::optional<std::vector<std::uint8_t>> segMaskData = message->getMaskData();
        if(segMaskData) {
            imgDetections->set_data((*segMaskData).data(), (*segMaskData).size());
        }
    }
    return imgDetections;
}

template <>
std::unique_ptr<google::protobuf::Message> getProtoMessage(const EncodedFrame* message, bool metadataOnly) {
    // Create a unique pointer to the protobuf EncodedFrame message
    auto encodedFrame = std::make_unique<proto::encoded_frame::EncodedFrame>();

    // Populate the protobuf message fields with the EncodedFrame data
    encodedFrame->set_instancenum(message->instanceNum);  // instanceNum -> instancenum
    encodedFrame->set_width(message->width);
    encodedFrame->set_height(message->height);
    encodedFrame->set_quality(message->quality);
    encodedFrame->set_bitrate(message->bitrate);
    encodedFrame->set_profile(static_cast<proto::encoded_frame::Profile>(message->profile));  // Profile enum
    encodedFrame->set_lossless(message->lossless);
    encodedFrame->set_type(static_cast<proto::encoded_frame::FrameType>(message->type));  // FrameType enum
    encodedFrame->set_frameoffset(message->frameOffset);                                  // frameOffset -> frameoffset
    encodedFrame->set_framesize(message->frameSize);                                      // frameSize -> framesize
    encodedFrame->set_sequencenum(message->sequenceNum);                                  // sequenceNum -> sequencenum

    // Set timestamps
    proto::common::Timestamp* ts = encodedFrame->mutable_ts();
    ts->set_sec(message->ts.sec);
    ts->set_nsec(message->ts.nsec);

    proto::common::Timestamp* tsDevice = encodedFrame->mutable_tsdevice();
    tsDevice->set_sec(message->tsDevice.sec);
    tsDevice->set_nsec(message->tsDevice.nsec);

    // Set camera settings
    proto::common::CameraSettings* cam = encodedFrame->mutable_cam();
    cam->set_exposuretimeus(message->cam.exposureTimeUs);    // exposureTimeUs -> exposuretimeus
    cam->set_sensitivityiso(message->cam.sensitivityIso);    // sensitivityIso -> sensitivityiso
    cam->set_lensposition(message->cam.lensPosition);        // lensPosition -> lensposition
    cam->set_wbcolortemp(message->cam.wbColorTemp);          // wbColorTemp -> wbcolortemp
    cam->set_lenspositionraw(message->cam.lensPositionRaw);  // lensPositionRaw -> lenspositionraw

    if(!metadataOnly) {
        // Set the encoded message data
        encodedFrame->set_data(message->data->getData().data(), message->data->getData().size());
    }

    proto::common::ImgTransformation* imgTransformation = encodedFrame->mutable_transformation();
    utility::serializeImgTransformation(imgTransformation, message->transformation);

    // Return the populated protobuf message
    return encodedFrame;
}
template <>
std::unique_ptr<google::protobuf::Message> getProtoMessage(const ImgFrame* message, bool metadataOnly) {
    // create and populate ImgFrame protobuf message
    auto imgFrame = std::make_unique<proto::img_frame::ImgFrame>();
    proto::common::Timestamp* ts = imgFrame->mutable_ts();
    ts->set_sec(message->ts.sec);
    ts->set_nsec(message->ts.nsec);
    proto::common::Timestamp* tsDevice = imgFrame->mutable_tsdevice();
    tsDevice->set_sec(message->tsDevice.sec);
    tsDevice->set_nsec(message->tsDevice.nsec);

    imgFrame->set_sequencenum(message->sequenceNum);

    proto::img_frame::Specs* fb = imgFrame->mutable_fb();
    fb->set_type(static_cast<proto::img_frame::Type>(message->fb.type));
    fb->set_width(message->fb.width);
    fb->set_height(message->fb.height);
    fb->set_stride(message->getStride());  // getStride() handles the case when fb.stride is set to 0
    fb->set_bytespp(message->fb.bytesPP);
    fb->set_p1offset(message->fb.p1Offset);
    fb->set_p2offset(message->fb.p2Offset);
    fb->set_p3offset(message->fb.p3Offset);

    proto::img_frame::Specs* sourceFb = imgFrame->mutable_sourcefb();
    sourceFb->set_type(static_cast<proto::img_frame::Type>(message->sourceFb.type));
    sourceFb->set_width(message->sourceFb.width);
    sourceFb->set_height(message->sourceFb.height);
    sourceFb->set_stride(message->sourceFb.stride);
    sourceFb->set_bytespp(message->sourceFb.bytesPP);
    sourceFb->set_p1offset(message->sourceFb.p1Offset);
    sourceFb->set_p2offset(message->sourceFb.p2Offset);
    sourceFb->set_p3offset(message->sourceFb.p3Offset);

    proto::common::CameraSettings* cam = imgFrame->mutable_cam();
    cam->set_exposuretimeus(message->cam.exposureTimeUs);
    cam->set_sensitivityiso(message->cam.sensitivityIso);
    cam->set_lensposition(message->cam.lensPosition);
    cam->set_wbcolortemp(message->cam.wbColorTemp);
    cam->set_lenspositionraw(message->cam.lensPositionRaw);

    imgFrame->set_instancenum(message->instanceNum);

    imgFrame->set_category(message->category);

    proto::common::ImgTransformation* imgTransformation = imgFrame->mutable_transformation();
    utility::serializeImgTransformation(imgTransformation, message->transformation);

    if(!metadataOnly) {
        imgFrame->set_data(message->data->getData().data(), message->data->getData().size());
    }

    return imgFrame;
}
template <>
std::unique_ptr<google::protobuf::Message> getProtoMessage(const SegmentationMask* message, bool metadataOnly) {
    auto segmentationMask = std::make_unique<proto::segmentation_mask::SegmentationMask>();

    segmentationMask->set_sequencenum(message->sequenceNum);

    auto timestamp = segmentationMask->mutable_ts();
    timestamp->set_sec(message->ts.sec);
    timestamp->set_nsec(message->ts.nsec);

    auto timestampDevice = segmentationMask->mutable_tsdevice();
    timestampDevice->set_sec(message->tsDevice.sec);
    timestampDevice->set_nsec(message->tsDevice.nsec);

    segmentationMask->set_width(message->getWidth());
    segmentationMask->set_height(message->getHeight());

    if(message->transformation.has_value()) {
        serializeImgTransformation(segmentationMask->mutable_transformation(), *message->transformation);
    }

    if(!metadataOnly) {
        segmentationMask->set_data(message->data->getData().data(), message->data->getSize());
    }

    return segmentationMask;
}
template <>
std::unique_ptr<google::protobuf::Message> getProtoMessage(const PointCloudData* message, bool metadataOnly) {
    auto pointCloudData = std::make_unique<dai::proto::point_cloud_data::PointCloudData>();

    auto timestamp = pointCloudData->mutable_ts();
    timestamp->set_sec(message->ts.sec);
    timestamp->set_nsec(message->ts.nsec);

    auto timestampDevice = pointCloudData->mutable_tsdevice();
    timestampDevice->set_sec(message->tsDevice.sec);
    timestampDevice->set_nsec(message->tsDevice.nsec);

    pointCloudData->set_sequencenum(message->sequenceNum);
    pointCloudData->set_width(message->getWidth());
    pointCloudData->set_height(message->getHeight());
    pointCloudData->set_instancenum(message->getInstanceNum());
    pointCloudData->set_minx(message->getMinX());
    pointCloudData->set_miny(message->getMinY());
    pointCloudData->set_minz(message->getMinZ());
    pointCloudData->set_maxx(message->getMaxX());
    pointCloudData->set_maxy(message->getMaxY());
    pointCloudData->set_maxz(message->getMaxZ());
    pointCloudData->set_sparse(message->isSparse());
    pointCloudData->set_color(message->isColor());

    if(!metadataOnly) {
        pointCloudData->set_data(message->data->getData().data(), message->data->getSize());
    }

    return pointCloudData;
}

// template <>
// void setProtoMessage(ImgAnnotations* obj, std::shared_ptr<google::protobuf::Message> msg, bool) {
// }
// template <>
// void setProtoMessage(SpatialImgDetections* obj, std::shared_ptr<google::protobuf::Message> msg, bool) {
// }
// template <>
// void setProtoMessage(ImgDetections* obj, std::shared_ptr<google::protobuf::Message> msg, bool) {
// }
template <>
void setProtoMessage(IMUData& obj, const google::protobuf::Message* msg, bool) {
    auto imuData = dynamic_cast<const proto::imu_data::IMUData*>(msg);
    obj.packets.clear();
    obj.packets.reserve(imuData->packets().size());
    for(auto packet : imuData->packets()) {
        IMUPacket imuPacket;
        auto protoAccelerometer = packet.accelerometer();
        auto& daiAccelerometer = imuPacket.acceleroMeter;
        daiAccelerometer.x = protoAccelerometer.vec().x();
        daiAccelerometer.y = protoAccelerometer.vec().y();
        daiAccelerometer.z = protoAccelerometer.vec().z();
        daiAccelerometer.accuracy = static_cast<IMUReport::Accuracy>(protoAccelerometer.report().accuracy());
        daiAccelerometer.sequence = protoAccelerometer.report().sequence();
        daiAccelerometer.timestamp.sec = protoAccelerometer.report().ts().sec();
        daiAccelerometer.timestamp.nsec = protoAccelerometer.report().ts().nsec();
        daiAccelerometer.tsDevice.sec = protoAccelerometer.report().tsdevice().sec();
        daiAccelerometer.tsDevice.nsec = protoAccelerometer.report().tsdevice().nsec();

        auto protoGyroscope = packet.gyroscope();
        auto& daiGyroscope = imuPacket.gyroscope;
        daiGyroscope.x = protoGyroscope.vec().x();
        daiGyroscope.y = protoGyroscope.vec().y();
        daiGyroscope.z = protoGyroscope.vec().z();
        daiGyroscope.accuracy = static_cast<IMUReport::Accuracy>(protoGyroscope.report().accuracy());
        daiGyroscope.sequence = protoGyroscope.report().sequence();
        daiGyroscope.timestamp.sec = protoGyroscope.report().ts().sec();
        daiGyroscope.timestamp.nsec = protoGyroscope.report().ts().nsec();
        daiGyroscope.tsDevice.sec = protoGyroscope.report().tsdevice().sec();
        daiGyroscope.tsDevice.nsec = protoGyroscope.report().tsdevice().nsec();

        auto protoMagnetometer = packet.magnetometer();
        auto& daiMagnetometer = imuPacket.magneticField;
        daiMagnetometer.x = protoMagnetometer.vec().x();
        daiMagnetometer.y = protoMagnetometer.vec().y();
        daiMagnetometer.z = protoMagnetometer.vec().z();
        daiMagnetometer.accuracy = static_cast<IMUReport::Accuracy>(protoMagnetometer.report().accuracy());
        daiMagnetometer.sequence = protoMagnetometer.report().sequence();
        daiMagnetometer.timestamp.sec = protoMagnetometer.report().ts().sec();
        daiMagnetometer.timestamp.nsec = protoMagnetometer.report().ts().nsec();
        daiMagnetometer.tsDevice.sec = protoMagnetometer.report().tsdevice().sec();
        daiMagnetometer.tsDevice.nsec = protoMagnetometer.report().tsdevice().nsec();

        auto protoRotationVector = packet.rotationvector();
        auto& daiRotationVector = imuPacket.rotationVector;
        daiRotationVector.i = protoRotationVector.quat().x();
        daiRotationVector.j = protoRotationVector.quat().y();
        daiRotationVector.k = protoRotationVector.quat().z();
        daiRotationVector.real = protoRotationVector.quat().w();
        daiRotationVector.accuracy = static_cast<IMUReport::Accuracy>(protoRotationVector.report().accuracy());
        daiRotationVector.sequence = protoRotationVector.report().sequence();
        daiRotationVector.timestamp.sec = protoRotationVector.report().ts().sec();
        daiRotationVector.timestamp.nsec = protoRotationVector.report().ts().nsec();
        daiRotationVector.tsDevice.sec = protoRotationVector.report().tsdevice().sec();
        daiRotationVector.tsDevice.nsec = protoRotationVector.report().tsdevice().nsec();

        obj.packets.push_back(imuPacket);
    }

    obj.setTimestamp(fromProtoTimestamp(imuData->ts()));
    obj.setTimestampDevice(fromProtoTimestamp(imuData->tsdevice()));
    obj.setSequenceNum(imuData->sequencenum());
}
template <>
void setProtoMessage(ImgFrame& obj, const google::protobuf::Message* msg, bool metadataOnly) {
    auto imgFrame = dynamic_cast<const proto::img_frame::ImgFrame*>(msg);
    // create and populate ImgFrame protobuf message
    obj.setTimestamp(utility::fromProtoTimestamp(imgFrame->ts()));
    obj.setTimestampDevice(utility::fromProtoTimestamp(imgFrame->tsdevice()));

    obj.setSequenceNum(imgFrame->sequencenum());

    obj.fb.type = static_cast<dai::ImgFrame::Type>(imgFrame->fb().type());
    obj.fb.width = imgFrame->fb().width();
    obj.fb.height = imgFrame->fb().height();
    obj.fb.stride = imgFrame->fb().stride();
    obj.fb.bytesPP = imgFrame->fb().bytespp();
    obj.fb.p1Offset = imgFrame->fb().p1offset();
    obj.fb.p2Offset = imgFrame->fb().p2offset();
    obj.fb.p3Offset = imgFrame->fb().p3offset();

    obj.sourceFb.type = static_cast<dai::ImgFrame::Type>(imgFrame->sourcefb().type());
    obj.sourceFb.width = imgFrame->sourcefb().width();
    obj.sourceFb.height = imgFrame->sourcefb().height();
    obj.sourceFb.stride = imgFrame->sourcefb().stride();
    obj.sourceFb.bytesPP = imgFrame->sourcefb().bytespp();
    obj.sourceFb.p1Offset = imgFrame->sourcefb().p1offset();
    obj.sourceFb.p2Offset = imgFrame->sourcefb().p2offset();
    obj.sourceFb.p3Offset = imgFrame->sourcefb().p3offset();

    obj.cam.exposureTimeUs = imgFrame->cam().exposuretimeus();
    obj.cam.sensitivityIso = imgFrame->cam().sensitivityiso();
    obj.cam.lensPosition = imgFrame->cam().lensposition();
    obj.cam.wbColorTemp = imgFrame->cam().wbcolortemp();
    obj.cam.lensPositionRaw = imgFrame->cam().lenspositionraw();

    obj.instanceNum = imgFrame->instancenum();

    obj.category = imgFrame->category();

    obj.transformation = deserializeImgTransformation(imgFrame->transformation());

    if(!metadataOnly) {
        std::vector<uint8_t> data(imgFrame->data().begin(), imgFrame->data().end());
        obj.setData(data);
    }
}
template <>
void setProtoMessage(EncodedFrame& obj, const google::protobuf::Message* msg, bool metadataOnly) {
    auto encFrame = dynamic_cast<const proto::encoded_frame::EncodedFrame*>(msg);
    // create and populate ImgFrame protobuf message
    obj.setTimestamp(utility::fromProtoTimestamp(encFrame->ts()));
    obj.setTimestampDevice(utility::fromProtoTimestamp(encFrame->tsdevice()));

    obj.setSequenceNum(encFrame->sequencenum());

    obj.width = encFrame->width();
    obj.height = encFrame->height();

    obj.instanceNum = encFrame->instancenum();

    obj.quality = encFrame->quality();
    obj.bitrate = encFrame->bitrate();
    obj.profile = static_cast<EncodedFrame::Profile>(encFrame->profile());

    obj.lossless = encFrame->lossless();
    obj.type = static_cast<EncodedFrame::FrameType>(encFrame->type());

    obj.frameOffset = encFrame->frameoffset();
    obj.frameSize = encFrame->framesize();

    obj.cam.exposureTimeUs = encFrame->cam().exposuretimeus();
    obj.cam.sensitivityIso = encFrame->cam().sensitivityiso();
    obj.cam.lensPosition = encFrame->cam().lensposition();
    obj.cam.wbColorTemp = encFrame->cam().wbcolortemp();
    obj.cam.lensPositionRaw = encFrame->cam().lenspositionraw();

    obj.transformation = deserializeImgTransformation(encFrame->transformation());

    if(!metadataOnly) {
        std::vector<uint8_t> data(encFrame->data().begin(), encFrame->data().end());
        obj.setData(data);
    }
}
template <>
void setProtoMessage(PointCloudData& obj, const google::protobuf::Message* msg, bool metadataOnly) {
    auto pcl = dynamic_cast<const proto::point_cloud_data::PointCloudData*>(msg);
    // create and populate ImgFrame protobuf message
    obj.setTimestamp(utility::fromProtoTimestamp(pcl->ts()));
    obj.setTimestampDevice(utility::fromProtoTimestamp(pcl->tsdevice()));

    obj.setSequenceNum(pcl->sequencenum());

    obj.setWidth(pcl->width());
    obj.setHeight(pcl->height());

    obj.setInstanceNum(pcl->instancenum());

    obj.setMinX(pcl->minx());
    obj.setMinY(pcl->miny());
    obj.setMinZ(pcl->minz());
    obj.setMaxX(pcl->maxx());
    obj.setMaxY(pcl->maxy());
    obj.setMaxZ(pcl->maxz());
    obj.setSparse(pcl->sparse());
    obj.setColor(pcl->color());

    if(!metadataOnly) {
        std::vector<uint8_t> data(pcl->data().begin(), pcl->data().end());
        obj.setData(data);
    }
}

};  // namespace utility
};  // namespace dai
