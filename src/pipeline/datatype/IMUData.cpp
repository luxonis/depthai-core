#include "depthai/pipeline/datatype/IMUData.hpp"

namespace dai {

IMUData::Serialized IMUData::serialize() const {
    return {data, raw};
}

IMUData::IMUData() : Buffer(std::make_shared<RawIMUData>()), rawIMU(*dynamic_cast<RawIMUData*>(raw.get())), packets(rawIMU.packets) {}
IMUData::IMUData(std::shared_ptr<RawIMUData> ptr) : Buffer(std::move(ptr)), rawIMU(*dynamic_cast<RawIMUData*>(raw.get())), packets(rawIMU.packets) {}

}  // namespace dai
