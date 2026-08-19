#include "ProtoSerialize.hpp"

#include <google/protobuf/descriptor.pb.h>
#include <google/protobuf/message.h>
#include <google/protobuf/util/time_util.h>

#include <chrono>
#include <cstdint>
#include <optional>
#include <queue>
#include <utility>
#include <variant>

#include "depthai/schemas/PointCloudData.pb.h"
#include "depthai/schemas/RGBDData.pb.h"
#include "depthai/schemas/SegmentationMask.pb.h"
#include "depthai/schemas/common.pb.h"
#include "pipeline/datatype/DatatypeEnum.hpp"
#include "pipeline/datatype/ImgDetections.hpp"
#include "pipeline/datatype/RGBDData.hpp"
#include "pipeline/datatype/SegmentationMask.hpp"

#ifndef DEPTHAI_PROTO_IMPL
    #define DEPTHAI_PROTO_IMPL(daiMsg, protoMsg)                                             \
        template <>                                                                          \
        ProtoSerializable::SchemaPair getProtoSchema<daiMsg>() {                             \
            static const ProtoSerializable::SchemaPair returnPair = [] {                     \
                protoMsg protoMessage;                                                       \
                const auto* descriptor = protoMessage.GetDescriptor();                       \
                if(descriptor == nullptr) {                                                  \
                    throw std::runtime_error("Failed to get protobuf descriptor");           \
                }                                                                            \
                ProtoSerializable::SchemaPair schemaPair;                                    \
                schemaPair.schemaName = descriptor->full_name();                             \
                schemaPair.schema = serializeFdSet(descriptor);                              \
                return schemaPair;                                                           \
            }();                                                                             \
            return returnPair;                                                               \
        }                                                                                    \
        void deserializeProtoMessage(daiMsg& obj, const std::vector<std::uint8_t>& bytes) {  \
            protoMsg protoMessage;                                                           \
            if(!protoMessage.ParseFromArray(bytes.data(), static_cast<int>(bytes.size()))) { \
                throw std::runtime_error("Failed to parse " #daiMsg " protobuf");            \
            }                                                                                \
            setProtoMessage(obj, &protoMessage);                                             \
        }
#endif

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

namespace {

void serializePoint2f(proto::common::Point2f* protoPoint, const Point2f& point) {
    protoPoint->set_x(point.x);
    protoPoint->set_y(point.y);
    if(point.hasNormalized) {
        protoPoint->set_normalized(point.normalized);
    } else {
        protoPoint->clear_normalized();
    }
}

Point2f deserializePoint2f(const proto::common::Point2f& point) {
    if(point.has_normalized()) {
        return Point2f{point.x(), point.y(), point.normalized()};
    }
    return Point2f{point.x(), point.y()};
}

void serializeSize2f(proto::common::Size2f* protoSize, const Size2f& size) {
    protoSize->set_width(size.width);
    protoSize->set_height(size.height);
    if(size.hasNormalized) {
        protoSize->set_normalized(size.normalized);
    } else {
        protoSize->clear_normalized();
    }
}

Size2f deserializeSize2f(const proto::common::Size2f& size) {
    if(size.has_normalized()) {
        return Size2f{size.width(), size.height(), size.normalized()};
    }
    return Size2f{size.width(), size.height()};
}

void serializeSpatialRect(proto::spatial_img_detections::Rect* protoRect, const Rect& rect) {
    protoRect->set_x(rect.x);
    protoRect->set_y(rect.y);
    protoRect->set_width(rect.width);
    protoRect->set_height(rect.height);
    if(rect.hasNormalized) {
        protoRect->set_normalized(rect.normalized);
    } else {
        protoRect->clear_normalized();
    }
}

Rect deserializeSpatialRect(const proto::spatial_img_detections::Rect& rect) {
    if(rect.has_normalized()) {
        return Rect{rect.x(), rect.y(), rect.width(), rect.height(), rect.normalized()};
    }
    return Rect{rect.x(), rect.y(), rect.width(), rect.height()};
}

}  // namespace

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

    const auto extrinsics = transformation.getExtrinsics();
    proto::common::Extrinsics* protoExtrinsics = imgTransformation->mutable_extrinsics();
    proto::common::TransformationMatrix* rotationMatrix = protoExtrinsics->mutable_rotationmatrix();
    for(const auto& row : extrinsics.rotationMatrix) {
        proto::common::FloatArray* floatArray = rotationMatrix->add_arrays();
        for(const auto& value : row) {
            floatArray->add_values(value);
        }
    }
    protoExtrinsics->mutable_translation()->set_x(extrinsics.translation.x);
    protoExtrinsics->mutable_translation()->set_y(extrinsics.translation.y);
    protoExtrinsics->mutable_translation()->set_z(extrinsics.translation.z);
    protoExtrinsics->mutable_spectranslation()->set_x(extrinsics.specTranslation.x);
    protoExtrinsics->mutable_spectranslation()->set_y(extrinsics.specTranslation.y);
    protoExtrinsics->mutable_spectranslation()->set_z(extrinsics.specTranslation.z);
    protoExtrinsics->set_tocamerasocket(static_cast<proto::common::CameraBoardSocket>(extrinsics.toCameraSocket));
    protoExtrinsics->set_lengthunit(static_cast<proto::common::LengthUnit>(extrinsics.lengthUnit));
    protoExtrinsics->set_todeviceid(extrinsics.toDeviceId);

    for(const auto& crop : transformation.getSrcCrops()) {
        auto* protoCrop = imgTransformation->add_srccrops();
        serializePoint2f(protoCrop->mutable_center(), crop.center);
        serializeSize2f(protoCrop->mutable_size(), crop.size);
        protoCrop->set_angle(crop.angle);
    }
}
ImgTransformation deserializeImgTransformation(const proto::common::ImgTransformation& imgTransformation) {
    std::array<std::array<float, 3>, 3> transformationMatrix;
    std::array<std::array<float, 3>, 3> sourceIntrinsicMatrix;
    std::vector<float> distortionCoefficients;
    Extrinsics extrinsics;
    const auto validateMatrix = [](const auto& matrix, const char* matrixName) {
        if(matrix.arrays_size() < 3) {
            throw std::runtime_error(std::string(matrixName) + " must contain at least three rows");
        }
        for(auto i = 0; i < 3; ++i) {
            if(matrix.arrays(i).values_size() < 3) {
                throw std::runtime_error(std::string(matrixName) + " must contain at least three columns per row");
            }
        }
    };
    validateMatrix(imgTransformation.transformationmatrix(), "Transformation matrix");
    validateMatrix(imgTransformation.sourceintrinsicmatrix(), "Source intrinsic matrix");

    distortionCoefficients.reserve(imgTransformation.distortioncoefficients().values_size());
    for(auto i = 0U; i < 3; ++i)
        for(auto j = 0U; j < 3; ++j) transformationMatrix[i][j] = imgTransformation.transformationmatrix().arrays(i).values(j);
    for(auto i = 0U; i < 3; ++i)
        for(auto j = 0U; j < 3; ++j) sourceIntrinsicMatrix[i][j] = imgTransformation.sourceintrinsicmatrix().arrays(i).values(j);
    for(auto i = 0; i < imgTransformation.distortioncoefficients().values_size(); ++i)
        distortionCoefficients.push_back(imgTransformation.distortioncoefficients().values(i));

    if(imgTransformation.has_extrinsics()) {
        const auto& protoExtrinsics = imgTransformation.extrinsics();
        const auto& protoRotation = protoExtrinsics.rotationmatrix();
        if(protoRotation.arrays_size() > 0) {
            extrinsics.rotationMatrix.clear();
            extrinsics.rotationMatrix.reserve(protoRotation.arrays_size());
            for(int i = 0; i < protoRotation.arrays_size(); ++i) {
                const auto& row = protoRotation.arrays(i);
                auto& rotationRow = extrinsics.rotationMatrix.emplace_back();
                rotationRow.reserve(row.values_size());
                for(int j = 0; j < row.values_size(); ++j) {
                    rotationRow.push_back(row.values(j));
                }
            }
        }
        if(protoExtrinsics.has_translation()) {
            const auto& t = protoExtrinsics.translation();
            extrinsics.translation = Point3f(t.x(), t.y(), t.z());
        }
        if(protoExtrinsics.has_spectranslation()) {
            const auto& t = protoExtrinsics.spectranslation();
            extrinsics.specTranslation = Point3f(t.x(), t.y(), t.z());
        }
        extrinsics.toCameraSocket = static_cast<CameraBoardSocket>(protoExtrinsics.tocamerasocket());
        extrinsics.toDeviceId = protoExtrinsics.todeviceid();
        if(protoExtrinsics.has_lengthunit()) {
            extrinsics.lengthUnit = static_cast<LengthUnit>(protoExtrinsics.lengthunit());
        } else {
            extrinsics.lengthUnit = LengthUnit::CENTIMETER;
        }
    }

    std::vector<dai::RotatedRect> srcCrops;
    srcCrops.reserve(imgTransformation.srccrops_size());
    for(const auto& crop : imgTransformation.srccrops()) {
        dai::Point2f center = deserializePoint2f(crop.center());
        dai::Size2f size = deserializeSize2f(crop.size());
        srcCrops.emplace_back(center, size, crop.angle());
    }
    ImgTransformation transformation;
    transformation = ImgTransformation(imgTransformation.srcwidth(),
                                       imgTransformation.srcheight(),
                                       sourceIntrinsicMatrix,
                                       static_cast<CameraModel>(imgTransformation.distortionmodel()),
                                       distortionCoefficients,
                                       extrinsics);
    if(transformation.isValid()) {
        transformation.addTransformation(transformationMatrix);
        if(!srcCrops.empty()) {
            transformation.setSize(imgTransformation.width(), imgTransformation.height());
            transformation.addSrcCrops(srcCrops);
        } else {
            transformation.addCrop(0, 0, imgTransformation.width(), imgTransformation.height());
        }
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
    } else if(schemaName == proto::segmentation_mask::SegmentationMask::descriptor()->full_name()) {
        return DatatypeEnum::SegmentationMask;
    } else if(schemaName == proto::point_cloud_data::PointCloudData::descriptor()->full_name()) {
        return DatatypeEnum::PointCloudData;
    } else if(schemaName == proto::spatial_img_detections::SpatialImgDetections::descriptor()->full_name()) {
        return DatatypeEnum::SpatialImgDetections;
    } else if(schemaName == proto::rgbd_data::RGBDData::descriptor()->full_name()) {
        return DatatypeEnum::RGBDData;
    } else {
        throw std::runtime_error("Unknown schema name: " + schemaName);
    }
}

bool deserializationSupported(DatatypeEnum datatype) {
    switch(datatype) {
        case DatatypeEnum::ImgAnnotations:
        case DatatypeEnum::ImgDetections:
        case DatatypeEnum::SpatialImgDetections:
        case DatatypeEnum::ImgFrame:
        case DatatypeEnum::EncodedFrame:
        case DatatypeEnum::IMUData:
        case DatatypeEnum::SegmentationMask:
        case DatatypeEnum::PointCloudData:
        case DatatypeEnum::RGBDData:
            return true;
        case DatatypeEnum::ADatatype:
        case DatatypeEnum::Buffer:
        case DatatypeEnum::Transformable:
        case DatatypeEnum::NNData:
        case DatatypeEnum::ImageManipConfig:
        case DatatypeEnum::CameraControl:
        case DatatypeEnum::GateControl:
        case DatatypeEnum::SystemInformation:
        case DatatypeEnum::SystemInformationRVC4:
        case DatatypeEnum::SpatialLocationCalculatorConfig:
        case DatatypeEnum::SegmentationParserConfig:
        case DatatypeEnum::SpatialLocationCalculatorData:
        case DatatypeEnum::EdgeDetectorConfig:
        case DatatypeEnum::AprilTagConfig:
        case DatatypeEnum::AprilTags:
        case DatatypeEnum::Tracklets:
        case DatatypeEnum::StereoDepthConfig:
        case DatatypeEnum::NeuralDepthConfig:
        case DatatypeEnum::GPUStereoConfig:
        case DatatypeEnum::FeatureTrackerConfig:
        case DatatypeEnum::ThermalConfig:
        case DatatypeEnum::ToFConfig:
        case DatatypeEnum::TrackedFeatures:
        case DatatypeEnum::BenchmarkReport:
        case DatatypeEnum::MapData:
        case DatatypeEnum::MessageGroup:
        case DatatypeEnum::TransformData:
        case DatatypeEnum::PointCloudConfig:
        case DatatypeEnum::ImageAlignConfig:
        case DatatypeEnum::ImageFiltersConfig:
        case DatatypeEnum::ToFDepthConfidenceFilterConfig:
        case DatatypeEnum::ObjectTrackerConfig:
        case DatatypeEnum::VppConfig:
        case DatatypeEnum::DynamicCalibrationControl:
        case DatatypeEnum::DynamicCalibrationResult:
        case DatatypeEnum::AutoCalibrationConfig:
        case DatatypeEnum::AutoCalibrationResult:
        case DatatypeEnum::CalibrationQuality:
        case DatatypeEnum::CalibrationMetrics:
        case DatatypeEnum::CoverageData:
        case DatatypeEnum::PipelineEvent:
        case DatatypeEnum::PipelineState:
        case DatatypeEnum::PipelineEventAggregationConfig:
        case DatatypeEnum::PacketizedData:
        case DatatypeEnum::ImgDetectionsFilterConfig:
        case DatatypeEnum::Classifications:
        case DatatypeEnum::Keypoints:
        case DatatypeEnum::Clusters:
        case DatatypeEnum::Map2D:
        case DatatypeEnum::Lines:
        case DatatypeEnum::Predictions:
        case DatatypeEnum::FastSAMParserConfig:
        case DatatypeEnum::HRNetParserConfig:
        case DatatypeEnum::MLSDParserConfig:
        case DatatypeEnum::MPPalmDetectionParserConfig:
        case DatatypeEnum::PPTextDetectionParserConfig:
        case DatatypeEnum::RFDETRParserConfig:
        case DatatypeEnum::SCRFDParserConfig:
        case DatatypeEnum::SuperAnimalParserConfig:
        case DatatypeEnum::YuNetParserConfig:
        case DatatypeEnum::ClassificationSequenceParserConfig:
        case DatatypeEnum::MapOutputParserConfig:
        case DatatypeEnum::XFeatMonoParserConfig:
        case DatatypeEnum::XFeatStereoParserConfig:
        case DatatypeEnum::COUNT:
            return false;
    }
    return false;
}

namespace {

template <typename ProtoMessageT, typename BufferT>
void populateBufferMetadata(BufferT& obj, const ProtoMessageT& proto) {
    obj.setTimestamp(safeTimestamp<std::chrono::steady_clock>(proto.ts(), proto.has_ts()));
    obj.setTimestampDevice(safeTimestamp<std::chrono::steady_clock>(proto.tsdevice(), proto.has_tsdevice()));

    if(proto.has_tssystem()) {
        obj.setTimestampSystem(fromProtoTimestamp<std::chrono::system_clock>(proto.tssystem()));
    } else {
        obj.setTimestampSystem(std::nullopt);
    }

    obj.setSequenceNum(proto.sequencenum());
}

template <typename ProtoPointT>
Point3f deserializePoint3f(const ProtoPointT& point) {
    return Point3f{point.x(), point.y(), point.z()};
}

Color deserializeColor(const proto::common::Color& color) {
    return Color{color.r(), color.g(), color.b(), color.a()};
}

RotatedRect deserializeRotatedRect(const proto::common::RotatedRect& bbox) {
    return RotatedRect{
        deserializePoint2f(bbox.center()),
        deserializeSize2f(bbox.size()),
        bbox.angle(),
    };
}

template <typename ProtoEdgesT>
std::vector<Edge> deserializeEdges(const ProtoEdgesT& protoEdges) {
    std::vector<Edge> edges;
    edges.reserve(protoEdges.size());
    for(const auto& protoEdge : protoEdges) {
        edges.push_back({protoEdge.src(), protoEdge.dst()});
    }
    return edges;
}

KeypointsList deserializeKeypointsList(const proto::common::KeypointsList& protoKeypoints) {
    std::vector<Keypoint> keypoints;
    keypoints.reserve(protoKeypoints.keypoints_size());
    for(const auto& protoKeypoint : protoKeypoints.keypoints()) {
        keypoints.emplace_back(
            deserializePoint3f(protoKeypoint.imagecoordinates()), protoKeypoint.confidence(), protoKeypoint.label(), protoKeypoint.labelname());
    }
    return KeypointsList(std::move(keypoints), deserializeEdges(protoKeypoints.edges()));
}

SpatialKeypointsList deserializeSpatialKeypointsList(const proto::common::SpatialKeypointsList& protoKeypoints) {
    std::vector<SpatialKeypoint> keypoints;
    keypoints.reserve(protoKeypoints.keypoints_size());
    for(const auto& protoKeypoint : protoKeypoints.keypoints()) {
        keypoints.emplace_back(deserializePoint3f(protoKeypoint.imagecoordinates()),
                               deserializePoint3f(protoKeypoint.spatialcoordinates()),
                               protoKeypoint.confidence(),
                               protoKeypoint.label(),
                               protoKeypoint.labelname());
    }
    return SpatialKeypointsList(std::move(keypoints), deserializeEdges(protoKeypoints.edges()), static_cast<LengthUnit>(protoKeypoints.unit()));
}

void populateImgDetection(ImgDetection& detection, const proto::img_detections::ImgDetection& protoDetection) {
    detection.label = protoDetection.label();
    detection.labelName = protoDetection.labelname();
    detection.confidence = protoDetection.confidence();
    detection.xmin = protoDetection.xmin();
    detection.ymin = protoDetection.ymin();
    detection.xmax = protoDetection.xmax();
    detection.ymax = protoDetection.ymax();

    if(protoDetection.has_boundingbox()) {
        detection.boundingBox = deserializeRotatedRect(protoDetection.boundingbox());
    } else {
        detection.boundingBox.reset();
    }

    if(protoDetection.has_keypoints()) {
        detection.keypoints = deserializeKeypointsList(protoDetection.keypoints());
    } else {
        detection.keypoints.reset();
    }
}

// Helper function to populate an EncodedFrame proto from an EncodedFrame object
void populateEncodedFrameToProto(proto::encoded_frame::EncodedFrame* encodedFrame, const EncodedFrame* message, bool metadataOnly) {
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

    if(message->tsSystem.has_value()) {
        proto::common::Timestamp* tsSystem = encodedFrame->mutable_tssystem();
        tsSystem->set_sec(message->tsSystem.value().sec);
        tsSystem->set_nsec(message->tsSystem.value().nsec);
    } else {
        encodedFrame->clear_tssystem();
    }

    // Set camera settings
    proto::common::CameraSettings* cam = encodedFrame->mutable_cam();
    cam->set_exposuretimeus(message->cam.exposureTimeUs);    // exposureTimeUs -> exposuretimeus
    cam->set_sensitivityiso(message->cam.sensitivityIso);    // sensitivityIso -> sensitivityiso
    cam->set_lensposition(message->cam.lensPosition);        // lensPosition -> lensposition
    cam->set_wbcolortemp(message->cam.wbColorTemp);          // wbColorTemp -> wbcolortemp
    cam->set_lenspositionraw(message->cam.lensPositionRaw);  // lensPositionRaw -> lenspositionraw
    cam->set_fsync(static_cast<proto::common::CameraFsync>(message->cam.fsync));
    cam->set_sensormode(message->cam.sensorMode);
    cam->set_fps(message->cam.fps);
    if(message->cam.sensorTemperatureC.has_value()) {
        cam->set_sensortemperaturec(*message->cam.sensorTemperatureC);
    }

    if(!metadataOnly) {
        // Set the encoded message data
        encodedFrame->set_data(message->data->getData().data(), message->data->getData().size());
    }

    proto::common::ImgTransformation* imgTransformation = encodedFrame->mutable_transformation();
    utility::serializeImgTransformation(imgTransformation, message->transformation);
}

// Helper function to populate an ImgFrame proto from an ImgFrame object
void populateImgFrameToProto(proto::img_frame::ImgFrame* imgFrame, const ImgFrame* message, bool metadataOnly) {
    proto::common::Timestamp* ts = imgFrame->mutable_ts();
    ts->set_sec(message->ts.sec);
    ts->set_nsec(message->ts.nsec);
    proto::common::Timestamp* tsDevice = imgFrame->mutable_tsdevice();
    tsDevice->set_sec(message->tsDevice.sec);
    tsDevice->set_nsec(message->tsDevice.nsec);

    if(message->tsSystem.has_value()) {
        proto::common::Timestamp* tsSystem = imgFrame->mutable_tssystem();
        tsSystem->set_sec(message->tsSystem.value().sec);
        tsSystem->set_nsec(message->tsSystem.value().nsec);
    } else {
        imgFrame->clear_tssystem();
    }

    imgFrame->set_sequencenum(message->sequenceNum);

    // frame buffer info
    proto::img_frame::Specs* fb = imgFrame->mutable_fb();
    fb->set_type(static_cast<proto::img_frame::Type>(message->fb.type));
    fb->set_width(message->fb.width);
    fb->set_height(message->fb.height);
    fb->set_stride(message->getStride());  // getStride() handles the case when fb.stride is set to 0
    fb->set_bytespp(message->fb.bytesPP);
    fb->set_p1offset(message->fb.p1Offset);
    fb->set_p2offset(message->fb.p2Offset);
    fb->set_p3offset(message->fb.p3Offset);

    // source frame buffer info
    proto::img_frame::Specs* sourceFb = imgFrame->mutable_sourcefb();
    sourceFb->set_type(static_cast<proto::img_frame::Type>(message->sourceFb.type));
    sourceFb->set_width(message->sourceFb.width);
    sourceFb->set_height(message->sourceFb.height);
    sourceFb->set_stride(message->sourceFb.stride);
    sourceFb->set_bytespp(message->sourceFb.bytesPP);
    sourceFb->set_p1offset(message->sourceFb.p1Offset);
    sourceFb->set_p2offset(message->sourceFb.p2Offset);
    sourceFb->set_p3offset(message->sourceFb.p3Offset);

    // camera settings
    proto::common::CameraSettings* cam = imgFrame->mutable_cam();
    cam->set_exposuretimeus(message->cam.exposureTimeUs);
    cam->set_sensitivityiso(message->cam.sensitivityIso);
    cam->set_lensposition(message->cam.lensPosition);
    cam->set_wbcolortemp(message->cam.wbColorTemp);
    cam->set_lenspositionraw(message->cam.lensPositionRaw);
    cam->set_fsync(static_cast<proto::common::CameraFsync>(message->cam.fsync));
    cam->set_sensormode(message->cam.sensorMode);
    cam->set_fps(message->cam.fps);
    if(message->cam.sensorTemperatureC.has_value()) {
        cam->set_sensortemperaturec(*message->cam.sensorTemperatureC);
    }

    // instance number and category
    imgFrame->set_instancenum(message->instanceNum);
    imgFrame->set_category(message->category);

    proto::common::ImgTransformation* imgTransformation = imgFrame->mutable_transformation();
    utility::serializeImgTransformation(imgTransformation, message->transformation);

    if(!metadataOnly) {
        imgFrame->set_data(message->data->getData().data(), message->data->getData().size());
    }
}

// Helper function to populate an EncodedFrame object from an EncodedFrame proto
void populateEncodedFrameFromProto(EncodedFrame& obj, const proto::encoded_frame::EncodedFrame& encFrame, bool metadataOnly) {
    const auto safeTimestamp = [](const auto& protoTs, bool hasField) {
        using clock = std::chrono::steady_clock;
        using steady_tp = std::chrono::time_point<clock>;
        return hasField ? utility::fromProtoTimestamp<clock>(protoTs) : steady_tp{};
    };

    obj.setTimestamp(safeTimestamp(encFrame.ts(), encFrame.has_ts()));
    obj.setTimestampDevice(safeTimestamp(encFrame.tsdevice(), encFrame.has_tsdevice()));

    if(encFrame.has_tssystem()) {
        obj.setTimestampSystem(utility::fromProtoTimestamp<std::chrono::system_clock>(encFrame.tssystem()));
    } else {
        obj.setTimestampSystem(std::nullopt);
    }

    obj.setSequenceNum(encFrame.sequencenum());

    obj.width = encFrame.width();
    obj.height = encFrame.height();

    obj.instanceNum = encFrame.instancenum();

    obj.quality = encFrame.quality();
    obj.bitrate = encFrame.bitrate();
    obj.profile = static_cast<EncodedFrame::Profile>(encFrame.profile());

    obj.lossless = encFrame.lossless();
    obj.type = static_cast<EncodedFrame::FrameType>(encFrame.type());

    obj.frameOffset = encFrame.frameoffset();
    obj.frameSize = encFrame.framesize();

    obj.cam.exposureTimeUs = encFrame.cam().exposuretimeus();
    obj.cam.sensitivityIso = encFrame.cam().sensitivityiso();
    obj.cam.lensPosition = encFrame.cam().lensposition();
    obj.cam.wbColorTemp = encFrame.cam().wbcolortemp();
    obj.cam.lensPositionRaw = encFrame.cam().lenspositionraw();
    obj.cam.fsync = static_cast<ImgFrame::Fsync>(encFrame.cam().fsync());
    obj.cam.sensorMode = encFrame.cam().sensormode();
    obj.cam.fps = encFrame.cam().fps();
    obj.cam.sensorTemperatureC = encFrame.cam().has_sensortemperaturec() ? std::make_optional(encFrame.cam().sensortemperaturec()) : std::nullopt;

    if(encFrame.has_transformation()) {
        obj.transformation = deserializeImgTransformation(encFrame.transformation());
    }

    if(!metadataOnly) {
        std::vector<uint8_t> data(encFrame.data().begin(), encFrame.data().end());
        obj.setData(std::move(data));
    }
}

// Helper function to populate an ImgFrame object from an ImgFrame proto
void populateImgFrameFromProto(ImgFrame& obj, const proto::img_frame::ImgFrame& imgFrame, bool metadataOnly) {
    const auto safeTimestamp = [](const auto& protoTs, bool hasField) {
        using steady_tp = std::chrono::time_point<std::chrono::steady_clock>;
        return hasField ? utility::fromProtoTimestamp<std::chrono::steady_clock>(protoTs) : steady_tp{};
    };
    obj.setTimestamp(safeTimestamp(imgFrame.ts(), imgFrame.has_ts()));
    obj.setTimestampDevice(safeTimestamp(imgFrame.tsdevice(), imgFrame.has_tsdevice()));

    if(imgFrame.has_tssystem()) {
        obj.setTimestampSystem(utility::fromProtoTimestamp<std::chrono::system_clock>(imgFrame.tssystem()));
    } else {
        obj.setTimestampSystem(std::nullopt);
    }

    obj.setSequenceNum(imgFrame.sequencenum());

    // frame buffer info
    obj.fb.type = static_cast<dai::ImgFrame::Type>(imgFrame.fb().type());
    obj.fb.width = imgFrame.fb().width();
    obj.fb.height = imgFrame.fb().height();
    obj.fb.stride = imgFrame.fb().stride();
    obj.fb.bytesPP = imgFrame.fb().bytespp();
    obj.fb.p1Offset = imgFrame.fb().p1offset();
    obj.fb.p2Offset = imgFrame.fb().p2offset();
    obj.fb.p3Offset = imgFrame.fb().p3offset();

    // source frame buffer info
    obj.sourceFb.type = static_cast<dai::ImgFrame::Type>(imgFrame.sourcefb().type());
    obj.sourceFb.width = imgFrame.sourcefb().width();
    obj.sourceFb.height = imgFrame.sourcefb().height();
    obj.sourceFb.stride = imgFrame.sourcefb().stride();
    obj.sourceFb.bytesPP = imgFrame.sourcefb().bytespp();
    obj.sourceFb.p1Offset = imgFrame.sourcefb().p1offset();
    obj.sourceFb.p2Offset = imgFrame.sourcefb().p2offset();
    obj.sourceFb.p3Offset = imgFrame.sourcefb().p3offset();

    // camera settings
    obj.cam.exposureTimeUs = imgFrame.cam().exposuretimeus();
    obj.cam.sensitivityIso = imgFrame.cam().sensitivityiso();
    obj.cam.lensPosition = imgFrame.cam().lensposition();
    obj.cam.wbColorTemp = imgFrame.cam().wbcolortemp();
    obj.cam.lensPositionRaw = imgFrame.cam().lenspositionraw();
    obj.cam.fsync = static_cast<ImgFrame::Fsync>(imgFrame.cam().fsync());
    obj.cam.sensorMode = imgFrame.cam().sensormode();
    obj.cam.fps = imgFrame.cam().fps();
    obj.cam.sensorTemperatureC = imgFrame.cam().has_sensortemperaturec() ? std::make_optional(imgFrame.cam().sensortemperaturec()) : std::nullopt;

    // instance number and category
    obj.instanceNum = imgFrame.instancenum();
    obj.category = imgFrame.category();

    if(imgFrame.has_transformation()) {
        obj.transformation = deserializeImgTransformation(imgFrame.transformation());
    }

    if(!metadataOnly) {
        std::vector<uint8_t> data(imgFrame.data().begin(), imgFrame.data().end());
        obj.setData(std::move(data));
    }
}

}  // namespace

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

    if(message->tsSystem.has_value()) {
        proto::common::Timestamp* tsSystem = imageAnnotations->mutable_tssystem();
        tsSystem->set_sec(message->tsSystem.value().sec);
        tsSystem->set_nsec(message->tsSystem.value().nsec);
    } else {
        imageAnnotations->clear_tssystem();
    }

    if(message->transformation.has_value()) {
        proto::common::ImgTransformation* imgTransformation = imageAnnotations->mutable_transformation();
        utility::serializeImgTransformation(imgTransformation, message->transformation.value());
    }

    for(const auto& annotation : message->annotations) {
        proto::image_annotations::ImageAnnotation* imageAnnotation = imageAnnotations->add_annotations();
        for(const auto& circle : annotation.circles) {
            proto::image_annotations::CircleAnnotation* circleAnnotation = imageAnnotation->add_circles();
            serializePoint2f(circleAnnotation->mutable_position(), circle.position);
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
                serializePoint2f(protoPoint, point);
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
            serializePoint2f(textAnnotation->mutable_position(), text.position);
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

    if(message->tsSystem.has_value()) {
        proto::common::Timestamp* tsSystem = spatialImgDetections->mutable_tssystem();
        tsSystem->set_sec(message->tsSystem.value().sec);
        tsSystem->set_nsec(message->tsSystem.value().nsec);
    } else {
        spatialImgDetections->clear_tssystem();
    }

    for(const auto& detection : message->detections) {
        auto* spatialImgDetection = spatialImgDetections->add_detections();

        // Populate embedded ImgDetection message.
        auto* protoDetection = spatialImgDetection->mutable_detection();
        protoDetection->set_label(detection.label);
        protoDetection->set_labelname(detection.labelName);
        protoDetection->set_confidence(detection.confidence);
        protoDetection->set_xmin(detection.xmin);
        protoDetection->set_ymin(detection.ymin);
        protoDetection->set_xmax(detection.xmax);
        protoDetection->set_ymax(detection.ymax);

        if(detection.boundingBox.has_value() || !(detection.xmin == 0.f && detection.xmax == 0.f && detection.ymin == 0.f && detection.ymax == 0.f)) {
            const auto bbox = detection.boundingBox.has_value() ? detection.boundingBox.value() : detection.getBoundingBox();
            auto* bboxProto = protoDetection->mutable_boundingbox();
            serializePoint2f(bboxProto->mutable_center(), bbox.center);
            serializeSize2f(bboxProto->mutable_size(), bbox.size);
            bboxProto->set_angle(bbox.angle);
        }

        // Populate SpatialImgDetection.Point3f
        auto* spatialCoordinates = spatialImgDetection->mutable_spatialcoordinates();
        spatialCoordinates->set_x(detection.spatialCoordinates.x);
        spatialCoordinates->set_y(detection.spatialCoordinates.y);
        spatialCoordinates->set_z(detection.spatialCoordinates.z);

        // Populate SpatialImgDetection.SpatialLocationCalculatorConfigData
        auto* boundingBoxMapping = spatialImgDetection->mutable_boundingboxmapping();

        // Populate SpatialImgDetection.SpatialLocationCalculatorConfigData.Rect
        serializeSpatialRect(boundingBoxMapping->mutable_roi(), detection.boundingBoxMapping.roi);

        // Populate SpatialImgDetection.SpatialLocationCalculatorConfigData.SpatialLocationCalculatorConfigThresholds
        auto* depthTresholds = boundingBoxMapping->mutable_depththresholds();
        depthTresholds->set_lowerthreshold(detection.boundingBoxMapping.depthThresholds.lowerThreshold);
        depthTresholds->set_upperthreshold(detection.boundingBoxMapping.depthThresholds.upperThreshold);

        // Populate SpatialImgDetection.SpatialLocationCalculatorConfigData.SpatialLocationCalculatorAlgorithm
        boundingBoxMapping->set_calculationalgorithm(
            static_cast<proto::spatial_img_detections::SpatialLocationCalculatorAlgorithm>(detection.boundingBoxMapping.calculationAlgorithm));

        // Populate SpatialImgDetection.SpatialLocationCalculatorConfigData.stepSize
        boundingBoxMapping->set_stepsize(detection.boundingBoxMapping.stepSize);

        if(detection.keypoints.has_value()) {
            const auto& keypointsList = detection.keypoints.value();
            const auto keypointsVec = keypointsList.getKeypoints();
            const auto edgesVec = keypointsList.getEdges();

            // Spatial keypoints with spatial coordinates.
            auto* protoSpatialKeypoints = spatialImgDetection->mutable_keypoints();
            protoSpatialKeypoints->set_unit(static_cast<proto::common::LengthUnit>(keypointsList.unit));
            for(const auto& keypoint : keypointsVec) {
                auto* protoKeypoint = protoSpatialKeypoints->add_keypoints();
                auto* coords = protoKeypoint->mutable_imagecoordinates();
                coords->set_x(keypoint.imageCoordinates.x);
                coords->set_y(keypoint.imageCoordinates.y);
                coords->set_z(keypoint.imageCoordinates.z);
                protoKeypoint->set_confidence(keypoint.confidence);
                protoKeypoint->set_label(keypoint.label);
                protoKeypoint->set_labelname(keypoint.labelName);

                auto* spatialCoords = protoKeypoint->mutable_spatialcoordinates();
                spatialCoords->set_x(keypoint.spatialCoordinates.x);
                spatialCoords->set_y(keypoint.spatialCoordinates.y);
                spatialCoords->set_z(keypoint.spatialCoordinates.z);
            }
            for(const auto& edge : edgesVec) {
                auto* protoEdge = protoSpatialKeypoints->add_edges();
                protoEdge->set_src(edge[0]);
                protoEdge->set_dst(edge[1]);
            }
        }
    }

    if(message->transformation.has_value()) {
        proto::common::ImgTransformation* imgTransformation = spatialImgDetections->mutable_transformation();
        utility::serializeImgTransformation(imgTransformation, message->transformation.value());
    }

    spatialImgDetections->set_segmentationmaskwidth(static_cast<std::int64_t>(message->getSegmentationMaskWidth()));
    spatialImgDetections->set_segmentationmaskheight(static_cast<std::int64_t>(message->getSegmentationMaskHeight()));
    spatialImgDetections->set_unit(static_cast<proto::common::LengthUnit>(message->unit));

    if(!metadataOnly) {
        std::optional<std::vector<std::uint8_t>> segMaskData = message->getMaskData();
        if(segMaskData) {
            spatialImgDetections->set_maskdata((*segMaskData).data(), (*segMaskData).size());
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

        if(packet.acceleroMeter.tsSystem.has_value()) {
            imuAccelerometer->mutable_report()->mutable_tssystem()->set_sec(packet.acceleroMeter.tsSystem.value().sec);
            imuAccelerometer->mutable_report()->mutable_tssystem()->set_nsec(packet.acceleroMeter.tsSystem.value().nsec);
        } else {
            imuAccelerometer->mutable_report()->clear_tssystem();
        }

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

        if(packet.gyroscope.tsSystem.has_value()) {
            imuGyroscope->mutable_report()->mutable_tssystem()->set_sec(packet.gyroscope.tsSystem.value().sec);
            imuGyroscope->mutable_report()->mutable_tssystem()->set_nsec(packet.gyroscope.tsSystem.value().nsec);
        } else {
            imuGyroscope->mutable_report()->clear_tssystem();
        }

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

        if(packet.magneticField.tsSystem.has_value()) {
            imuMagnetometer->mutable_report()->mutable_tssystem()->set_sec(packet.magneticField.tsSystem.value().sec);
            imuMagnetometer->mutable_report()->mutable_tssystem()->set_nsec(packet.magneticField.tsSystem.value().nsec);
        } else {
            imuMagnetometer->mutable_report()->clear_tssystem();
        }

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

        if(packet.rotationVector.tsSystem.has_value()) {
            imuRotationVector->mutable_report()->mutable_tssystem()->set_sec(packet.rotationVector.tsSystem.value().sec);
            imuRotationVector->mutable_report()->mutable_tssystem()->set_nsec(packet.rotationVector.tsSystem.value().nsec);
        } else {
            imuRotationVector->mutable_report()->clear_tssystem();
        }
    }

    // Set timestamps
    proto::common::Timestamp* ts = imuData->mutable_ts();
    ts->set_sec(message->ts.sec);
    ts->set_nsec(message->ts.nsec);
    proto::common::Timestamp* tsDevice = imuData->mutable_tsdevice();
    tsDevice->set_sec(message->tsDevice.sec);
    tsDevice->set_nsec(message->tsDevice.nsec);

    if(message->tsSystem.has_value()) {
        proto::common::Timestamp* tsSystem = imuData->mutable_tssystem();
        tsSystem->set_sec(message->tsSystem.value().sec);
        tsSystem->set_nsec(message->tsSystem.value().nsec);
    } else {
        imuData->clear_tssystem();
    }

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

    if(message->tsSystem.has_value()) {
        proto::common::Timestamp* tsSystem = imgDetections->mutable_tssystem();
        tsSystem->set_sec(message->tsSystem.value().sec);
        tsSystem->set_nsec(message->tsSystem.value().nsec);
    } else {
        imgDetections->clear_tssystem();
    }

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
            serializePoint2f(bboxProto->mutable_center(), bbox.center);
            serializeSize2f(bboxProto->mutable_size(), bbox.size);
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
                protoKeypoint->set_labelname(keypoint.labelName);
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

    if(message->transformation.has_value()) {
        proto::common::ImgTransformation* imgTransformation = imgDetections->mutable_transformation();
        utility::serializeImgTransformation(imgTransformation, message->transformation.value());
    }

    if(!metadataOnly) {
        std::optional<std::vector<std::uint8_t>> segMaskData = message->getMaskData();
        if(segMaskData) {
            imgDetections->set_maskdata((*segMaskData).data(), (*segMaskData).size());
        }
    }
    return imgDetections;
}

template <>
std::unique_ptr<google::protobuf::Message> getProtoMessage(const EncodedFrame* message, bool metadataOnly) {
    auto encodedFrame = std::make_unique<proto::encoded_frame::EncodedFrame>();
    populateEncodedFrameToProto(encodedFrame.get(), message, metadataOnly);
    return encodedFrame;
}

template <>
std::unique_ptr<google::protobuf::Message> getProtoMessage(const ImgFrame* message, bool metadataOnly) {
    auto imgFrame = std::make_unique<proto::img_frame::ImgFrame>();
    populateImgFrameToProto(imgFrame.get(), message, metadataOnly);
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

    if(message->tsSystem.has_value()) {
        auto timestampSystem = segmentationMask->mutable_tssystem();
        timestampSystem->set_sec(message->tsSystem.value().sec);
        timestampSystem->set_nsec(message->tsSystem.value().nsec);
    } else {
        segmentationMask->clear_tssystem();
    }

    segmentationMask->set_width(message->getWidth());
    segmentationMask->set_height(message->getHeight());

    if(message->transformation.has_value()) {
        serializeImgTransformation(segmentationMask->mutable_transformation(), *message->transformation);
    }

    const auto labels = message->getLabels();
    for(const auto& label : labels) {
        segmentationMask->add_labels(label);
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

    if(message->tsSystem.has_value()) {
        auto timestampSystem = pointCloudData->mutable_tssystem();
        timestampSystem->set_sec(message->tsSystem.value().sec);
        timestampSystem->set_nsec(message->tsSystem.value().nsec);
    } else {
        pointCloudData->clear_tssystem();
    }

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

    // Set sparse flag based on height for backward compatibility with protobuf
    pointCloudData->set_sparse(message->getHeight() == 1);
    pointCloudData->set_color(message->isColor());

    if(!metadataOnly) {
        pointCloudData->set_data(message->data->getData().data(), message->data->getSize());
    }

    return pointCloudData;
}

template <>
std::unique_ptr<google::protobuf::Message> getProtoMessage(const RGBDData* message, bool metadataOnly) {
    auto rgbdData = std::make_unique<dai::proto::rgbd_data::RGBDData>();

    // Set timestamps
    auto timestamp = rgbdData->mutable_ts();
    timestamp->set_sec(message->ts.sec);
    timestamp->set_nsec(message->ts.nsec);

    auto timestampDevice = rgbdData->mutable_tsdevice();
    timestampDevice->set_sec(message->tsDevice.sec);
    timestampDevice->set_nsec(message->tsDevice.nsec);

    if(message->tsSystem.has_value()) {
        auto timestampSystem = rgbdData->mutable_tssystem();
        timestampSystem->set_sec(message->tsSystem.value().sec);
        timestampSystem->set_nsec(message->tsSystem.value().nsec);
    } else {
        rgbdData->clear_tssystem();
    }

    // Set sequence number
    rgbdData->set_sequencenum(message->sequenceNum);

    // Serialize color frame if present (can be ImgFrame or EncodedFrame)
    auto colorFrame = message->getRGBFrame();
    if(colorFrame.has_value()) {
        auto handler = [&](auto&& framePtr) {
            using T = std::decay_t<decltype(framePtr)>;
            if constexpr(std::is_same_v<T, std::shared_ptr<ImgFrame>>) {
                if(framePtr) {
                    populateImgFrameToProto(rgbdData->mutable_colorimgframe(), framePtr.get(), metadataOnly);
                }
            } else if constexpr(std::is_same_v<T, std::shared_ptr<EncodedFrame>>) {
                if(framePtr) {
                    populateEncodedFrameToProto(rgbdData->mutable_colorencodedframe(), framePtr.get(), metadataOnly);
                }
            } else {
                static_assert(sizeof(T*) == 0, "Unhandled frame type in RGBDData color frame variant");
            }
        };

        std::visit(handler, colorFrame.value());
    }

    // Serialize depth frame if present (can be ImgFrame or EncodedFrame)
    auto depthFrame = message->getDepthFrame();
    if(depthFrame.has_value()) {
        auto handler = [&](auto&& framePtr) {
            using T = std::decay_t<decltype(framePtr)>;
            if constexpr(std::is_same_v<T, std::shared_ptr<ImgFrame>>) {
                if(framePtr) {
                    populateImgFrameToProto(rgbdData->mutable_depthimgframe(), framePtr.get(), metadataOnly);
                }
            } else if constexpr(std::is_same_v<T, std::shared_ptr<EncodedFrame>>) {
                if(framePtr) {
                    populateEncodedFrameToProto(rgbdData->mutable_depthencodedframe(), framePtr.get(), metadataOnly);
                }
            } else {
                static_assert(sizeof(T*) == 0, "Unhandled frame type in RGBDData depth frame variant");
            }
        };

        std::visit(handler, depthFrame.value());
    }

    return rgbdData;
}

template <>
void setProtoMessage(ImgAnnotations& obj, const google::protobuf::Message* msg, bool) {
    auto imageAnnotations = dynamic_cast<const proto::image_annotations::ImageAnnotations*>(msg);
    if(imageAnnotations == nullptr) {
        throw std::runtime_error("Failed to cast protobuf message to ImgAnnotations");
    }

    populateBufferMetadata(obj, *imageAnnotations);
    obj.transformation =
        imageAnnotations->has_transformation() ? std::make_optional(deserializeImgTransformation(imageAnnotations->transformation())) : std::nullopt;

    obj.annotations.clear();
    obj.annotations.reserve(imageAnnotations->annotations_size());
    for(const auto& protoAnnotation : imageAnnotations->annotations()) {
        ImgAnnotation annotation;

        annotation.circles.reserve(protoAnnotation.circles_size());
        for(const auto& protoCircle : protoAnnotation.circles()) {
            CircleAnnotation circle;
            circle.position = deserializePoint2f(protoCircle.position());
            circle.diameter = protoCircle.diameter();
            circle.thickness = protoCircle.thickness();
            circle.fillColor = deserializeColor(protoCircle.fillcolor());
            circle.outlineColor = deserializeColor(protoCircle.outlinecolor());
            annotation.circles.push_back(circle);
        }

        annotation.points.reserve(protoAnnotation.points_size());
        for(const auto& protoPoints : protoAnnotation.points()) {
            PointsAnnotation points;
            points.type = static_cast<PointsAnnotationType>(protoPoints.type());
            points.points.reserve(protoPoints.points_size());
            for(const auto& protoPoint : protoPoints.points()) {
                points.points.push_back(deserializePoint2f(protoPoint));
            }
            points.outlineColor = deserializeColor(protoPoints.outlinecolor());
            points.outlineColors.reserve(protoPoints.outlinecolors_size());
            for(const auto& protoColor : protoPoints.outlinecolors()) {
                points.outlineColors.push_back(deserializeColor(protoColor));
            }
            points.fillColor = deserializeColor(protoPoints.fillcolor());
            points.thickness = protoPoints.thickness();
            annotation.points.push_back(points);
        }

        annotation.texts.reserve(protoAnnotation.texts_size());
        for(const auto& protoText : protoAnnotation.texts()) {
            TextAnnotation text;
            text.position = deserializePoint2f(protoText.position());
            text.text = protoText.text();
            text.fontSize = protoText.fontsize();
            text.textColor = deserializeColor(protoText.textcolor());
            text.backgroundColor = deserializeColor(protoText.backgroundcolor());
            annotation.texts.push_back(text);
        }

        obj.annotations.push_back(std::move(annotation));
    }
}

template <>
void setProtoMessage(SpatialImgDetections& obj, const google::protobuf::Message* msg, bool metadataOnly) {
    auto spatialImgDetections = dynamic_cast<const proto::spatial_img_detections::SpatialImgDetections*>(msg);
    if(spatialImgDetections == nullptr) {
        throw std::runtime_error("Failed to cast protobuf message to SpatialImgDetections");
    }

    populateBufferMetadata(obj, *spatialImgDetections);
    obj.transformation =
        spatialImgDetections->has_transformation() ? std::make_optional(deserializeImgTransformation(spatialImgDetections->transformation())) : std::nullopt;
    obj.unit = static_cast<LengthUnit>(spatialImgDetections->unit());
    obj.segmentationMaskWidth = static_cast<size_t>(spatialImgDetections->segmentationmaskwidth());
    obj.segmentationMaskHeight = static_cast<size_t>(spatialImgDetections->segmentationmaskheight());

    obj.detections.clear();
    obj.detections.reserve(spatialImgDetections->detections_size());
    for(const auto& protoSpatialDetection : spatialImgDetections->detections()) {
        SpatialImgDetection detection;

        if(protoSpatialDetection.has_detection()) {
            const auto& protoDetection = protoSpatialDetection.detection();
            detection.label = protoDetection.label();
            detection.labelName = protoDetection.labelname();
            detection.confidence = protoDetection.confidence();
            detection.xmin = protoDetection.xmin();
            detection.ymin = protoDetection.ymin();
            detection.xmax = protoDetection.xmax();
            detection.ymax = protoDetection.ymax();

            if(protoDetection.has_boundingbox()) {
                detection.boundingBox = deserializeRotatedRect(protoDetection.boundingbox());
            } else {
                detection.boundingBox.reset();
            }
        }

        detection.spatialCoordinates = deserializePoint3f(protoSpatialDetection.spatialcoordinates());

        if(protoSpatialDetection.has_boundingboxmapping()) {
            const auto& protoMapping = protoSpatialDetection.boundingboxmapping();
            detection.boundingBoxMapping.roi = deserializeSpatialRect(protoMapping.roi());
            detection.boundingBoxMapping.depthThresholds.lowerThreshold = protoMapping.depththresholds().lowerthreshold();
            detection.boundingBoxMapping.depthThresholds.upperThreshold = protoMapping.depththresholds().upperthreshold();
            detection.boundingBoxMapping.calculationAlgorithm = static_cast<SpatialLocationCalculatorAlgorithm>(protoMapping.calculationalgorithm());
            detection.boundingBoxMapping.stepSize = protoMapping.stepsize();
        }

        if(protoSpatialDetection.has_keypoints()) {
            detection.keypoints = deserializeSpatialKeypointsList(protoSpatialDetection.keypoints());
        } else {
            detection.keypoints.reset();
        }

        obj.detections.push_back(std::move(detection));
    }

    if(!metadataOnly) {
        if(!spatialImgDetections->maskdata().empty() && spatialImgDetections->maskdata().data() != nullptr) {
            std::vector<std::uint8_t> maskData(spatialImgDetections->maskdata().begin(), spatialImgDetections->maskdata().end());
            obj.setSegmentationMask(maskData, obj.segmentationMaskWidth, obj.segmentationMaskHeight);
        } else {
            std::vector<std::uint8_t> emptyData;
            obj.setData(std::move(emptyData));
        }
    }
}

template <>
void setProtoMessage(ImgDetections& obj, const google::protobuf::Message* msg, bool metadataOnly) {
    auto imgDetections = dynamic_cast<const proto::img_detections::ImgDetections*>(msg);
    if(imgDetections == nullptr) {
        throw std::runtime_error("Failed to cast protobuf message to ImgDetections");
    }

    populateBufferMetadata(obj, *imgDetections);
    obj.transformation = imgDetections->has_transformation() ? std::make_optional(deserializeImgTransformation(imgDetections->transformation())) : std::nullopt;
    obj.segmentationMaskWidth = static_cast<size_t>(imgDetections->segmentationmaskwidth());
    obj.segmentationMaskHeight = static_cast<size_t>(imgDetections->segmentationmaskheight());

    obj.detections.clear();
    obj.detections.reserve(imgDetections->detections_size());
    for(const auto& protoDetection : imgDetections->detections()) {
        ImgDetection detection;
        populateImgDetection(detection, protoDetection);
        obj.detections.push_back(std::move(detection));
    }

    if(!metadataOnly) {
        if(!imgDetections->maskdata().empty() && imgDetections->maskdata().data() != nullptr) {
            std::vector<std::uint8_t> maskData(imgDetections->maskdata().begin(), imgDetections->maskdata().end());
            obj.setSegmentationMask(maskData, obj.segmentationMaskWidth, obj.segmentationMaskHeight);
        } else {
            std::vector<std::uint8_t> emptyData;
            obj.setData(std::move(emptyData));
        }
    }
}

template <>
void setProtoMessage(IMUData& obj, const google::protobuf::Message* msg, bool) {
    auto imuData = dynamic_cast<const proto::imu_data::IMUData*>(msg);
    if(imuData == nullptr) {
        throw std::runtime_error("Failed to cast protobuf message to IMUData");
    }
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

        if(protoAccelerometer.report().has_tssystem()) {
            daiAccelerometer.tsSystem.emplace();
            daiAccelerometer.tsSystem->sec = protoAccelerometer.report().tssystem().sec();
            daiAccelerometer.tsSystem->nsec = protoAccelerometer.report().tssystem().nsec();
        } else {
            daiAccelerometer.tsSystem.reset();
        }

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

        if(protoGyroscope.report().has_tssystem()) {
            daiGyroscope.tsSystem.emplace();
            daiGyroscope.tsSystem->sec = protoGyroscope.report().tssystem().sec();
            daiGyroscope.tsSystem->nsec = protoGyroscope.report().tssystem().nsec();
        } else {
            daiGyroscope.tsSystem.reset();
        }

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

        if(protoMagnetometer.report().has_tssystem()) {
            daiMagnetometer.tsSystem.emplace();
            daiMagnetometer.tsSystem->sec = protoMagnetometer.report().tssystem().sec();
            daiMagnetometer.tsSystem->nsec = protoMagnetometer.report().tssystem().nsec();
        } else {
            daiMagnetometer.tsSystem.reset();
        }

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

        if(protoRotationVector.report().has_tssystem()) {
            daiRotationVector.tsSystem.emplace();
            daiRotationVector.tsSystem->sec = protoRotationVector.report().tssystem().sec();
            daiRotationVector.tsSystem->nsec = protoRotationVector.report().tssystem().nsec();
        } else {
            daiRotationVector.tsSystem.reset();
        }

        obj.packets.push_back(imuPacket);
    }

    obj.setTimestamp(fromProtoTimestamp<std::chrono::steady_clock>(imuData->ts()));
    obj.setTimestampDevice(fromProtoTimestamp<std::chrono::steady_clock>(imuData->tsdevice()));

    if(imuData->has_tssystem()) {
        obj.setTimestampSystem(fromProtoTimestamp<std::chrono::system_clock>(imuData->tssystem()));
    } else {
        obj.setTimestampSystem(std::nullopt);
    }

    obj.setSequenceNum(imuData->sequencenum());
}
template <>
void setProtoMessage(ImgFrame& obj, const google::protobuf::Message* msg, bool metadataOnly) {
    auto imgFrame = dynamic_cast<const proto::img_frame::ImgFrame*>(msg);
    if(imgFrame == nullptr) {
        throw std::runtime_error("Failed to cast protobuf message to ImgFrame");
    }
    populateImgFrameFromProto(obj, *imgFrame, metadataOnly);
}
template <>
void setProtoMessage(SegmentationMask& obj, const google::protobuf::Message* msg, bool metadataOnly) {
    auto segmentationMask = dynamic_cast<const proto::segmentation_mask::SegmentationMask*>(msg);
    if(segmentationMask == nullptr) {
        throw std::runtime_error("Failed to cast protobuf message to SegmentationMask");
    }

    populateBufferMetadata(obj, *segmentationMask);
    obj.transformation =
        segmentationMask->has_transformation() ? std::make_optional(deserializeImgTransformation(segmentationMask->transformation())) : std::nullopt;
    obj.setSize(static_cast<size_t>(segmentationMask->width()), static_cast<size_t>(segmentationMask->height()));
    obj.setLabels(std::vector<std::string>(segmentationMask->labels().begin(), segmentationMask->labels().end()));

    if(!metadataOnly) {
        if(!segmentationMask->data().empty() && segmentationMask->data().data() != nullptr) {
            obj.setMask(span<const std::uint8_t>(reinterpret_cast<const std::uint8_t*>(segmentationMask->data().data()), segmentationMask->data().size()),
                        obj.getWidth(),
                        obj.getHeight());
        } else {
            std::vector<std::uint8_t> emptyData;
            obj.setData(std::move(emptyData));
        }
    }
}

template <>
void setProtoMessage(EncodedFrame& obj, const google::protobuf::Message* msg, bool metadataOnly) {
    auto encFrame = dynamic_cast<const proto::encoded_frame::EncodedFrame*>(msg);
    if(encFrame == nullptr) {
        throw std::runtime_error("Failed to cast protobuf message to EncodedFrame");
    }
    populateEncodedFrameFromProto(obj, *encFrame, metadataOnly);
}
template <>
void setProtoMessage(PointCloudData& obj, const google::protobuf::Message* msg, bool metadataOnly) {
    auto pcl = dynamic_cast<const proto::point_cloud_data::PointCloudData*>(msg);
    if(pcl == nullptr) {
        throw std::runtime_error("Failed to cast protobuf message to PointCloudData");
    }
    const auto safeTimestamp = [](const auto& protoTs, bool hasField) {
        using steady_tp = std::chrono::time_point<std::chrono::steady_clock>;
        return hasField ? utility::fromProtoTimestamp<std::chrono::steady_clock>(protoTs) : steady_tp{};
    };
    // create and populate ImgFrame protobuf message
    obj.setTimestamp(safeTimestamp(pcl->ts(), pcl->has_ts()));
    obj.setTimestampDevice(safeTimestamp(pcl->tsdevice(), pcl->has_tsdevice()));

    if(pcl->has_tssystem()) {
        obj.setTimestampSystem(utility::fromProtoTimestamp<std::chrono::system_clock>(pcl->tssystem()));
    } else {
        obj.setTimestampSystem(std::nullopt);
    }

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
    obj.setColor(pcl->color());

    if(!metadataOnly) {
        std::vector<uint8_t> data(pcl->data().begin(), pcl->data().end());
        obj.setData(std::move(data));
    }
}

template <>
void setProtoMessage(RGBDData& obj, const google::protobuf::Message* msg, bool metadataOnly) {
    auto rgbdData = dynamic_cast<const proto::rgbd_data::RGBDData*>(msg);
    if(rgbdData == nullptr) {
        throw std::runtime_error("Failed to cast protobuf message to RGBDData");
    }

    obj.setTimestamp(utility::fromProtoTimestamp<std::chrono::steady_clock>(rgbdData->ts()));
    obj.setTimestampDevice(utility::fromProtoTimestamp<std::chrono::steady_clock>(rgbdData->tsdevice()));

    if(rgbdData->has_tssystem()) {
        obj.setTimestampSystem(utility::fromProtoTimestamp<std::chrono::system_clock>(rgbdData->tssystem()));
    } else {
        obj.setTimestampSystem(std::nullopt);
    }

    obj.setSequenceNum(rgbdData->sequencenum());

    // Deserialize color frame if present (can be ImgFrame or EncodedFrame)
    switch(rgbdData->color_frame_case()) {
        case proto::rgbd_data::RGBDData::kColorImgFrame: {
            auto colorFrame = std::make_shared<ImgFrame>();
            populateImgFrameFromProto(*colorFrame, rgbdData->colorimgframe(), metadataOnly);
            obj.setRGBFrame(colorFrame);
            break;
        }
        case proto::rgbd_data::RGBDData::kColorEncodedFrame: {
            auto colorFrame = std::make_shared<EncodedFrame>();
            populateEncodedFrameFromProto(*colorFrame, rgbdData->colorencodedframe(), metadataOnly);
            obj.setRGBFrame(colorFrame);
            break;
        }
        case proto::rgbd_data::RGBDData::COLOR_FRAME_NOT_SET:
            obj.setRGBFrame(std::shared_ptr<ImgFrame>{});
            break;
    }

    // Deserialize depth frame if present (can be ImgFrame or EncodedFrame)
    switch(rgbdData->depth_frame_case()) {
        case proto::rgbd_data::RGBDData::kDepthImgFrame: {
            auto depthFrame = std::make_shared<ImgFrame>();
            populateImgFrameFromProto(*depthFrame, rgbdData->depthimgframe(), metadataOnly);
            obj.setDepthFrame(depthFrame);
            break;
        }
        case proto::rgbd_data::RGBDData::kDepthEncodedFrame: {
            auto depthFrame = std::make_shared<EncodedFrame>();
            populateEncodedFrameFromProto(*depthFrame, rgbdData->depthencodedframe(), metadataOnly);
            obj.setDepthFrame(depthFrame);
            break;
        }
        case proto::rgbd_data::RGBDData::DEPTH_FRAME_NOT_SET:
            obj.setDepthFrame(std::shared_ptr<ImgFrame>{});
            break;
    }
}

DEPTHAI_PROTO_IMPL(ImgAnnotations, proto::image_annotations::ImageAnnotations)
DEPTHAI_PROTO_IMPL(SpatialImgDetections, proto::spatial_img_detections::SpatialImgDetections)
DEPTHAI_PROTO_IMPL(IMUData, proto::imu_data::IMUData)
DEPTHAI_PROTO_IMPL(ImgDetections, proto::img_detections::ImgDetections)
DEPTHAI_PROTO_IMPL(EncodedFrame, proto::encoded_frame::EncodedFrame)
DEPTHAI_PROTO_IMPL(ImgFrame, proto::img_frame::ImgFrame)
DEPTHAI_PROTO_IMPL(SegmentationMask, proto::segmentation_mask::SegmentationMask)
DEPTHAI_PROTO_IMPL(PointCloudData, proto::point_cloud_data::PointCloudData)
DEPTHAI_PROTO_IMPL(RGBDData, proto::rgbd_data::RGBDData)
#undef DEPTHAI_PROTO_IMPL

};  // namespace utility
};  // namespace dai
