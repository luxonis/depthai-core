#include "depthai/pipeline/datatype/IMUData.hpp"
#include "depthai/schemas/IMUData.pb.h"
#include "depthai/schemas/common.pb.h"

namespace dai {

std::unique_ptr<google::protobuf::Message> IMUData::getProtoMessage(bool) const {
    // create and populate ImgFrame protobuf message
    auto imuData = std::make_unique<proto::imu_data::IMUData>();
    auto imuPackets = imuData->mutable_packets();
    imuPackets->Reserve(packets.size());
    for(auto packet : packets) {
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
    return imuData;
}

void IMUData::setProtoMessage(const google::protobuf::Message& msg, bool) {
    auto imuData = dynamic_cast<const proto::imu_data::IMUData&>(msg);
    packets.clear();
    packets.reserve(imuData.packets().size());
    for(auto packet : imuData.packets()) {
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

        packets.push_back(imuPacket);
    }

    auto maxTimestamp = packets.size() > 0 ? packets.front().acceleroMeter.tsDevice.get().time_since_epoch() : std::chrono::nanoseconds{0};
    auto maxSeqNo = packets.size() > 0 ? packets.front().acceleroMeter.sequence : 0;
    for(const auto& packet : packets) {
        std::max({maxTimestamp,
                  packet.rotationVector.tsDevice.get().time_since_epoch(),
                  packet.gyroscope.tsDevice.get().time_since_epoch(),
                  packet.acceleroMeter.tsDevice.get().time_since_epoch(),
                  packet.magneticField.tsDevice.get().time_since_epoch()});
        std::max({maxSeqNo,
                  packet.rotationVector.sequence,
                  packet.gyroscope.sequence,
                  packet.acceleroMeter.sequence,
                  packet.magneticField.sequence});
    }
    tsDevice.sec = std::chrono::duration_cast<std::chrono::seconds>(maxTimestamp).count();
    tsDevice.nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(maxTimestamp).count() % 1000000000;
    sequenceNum = maxSeqNo;
}

}  // namespace dai
