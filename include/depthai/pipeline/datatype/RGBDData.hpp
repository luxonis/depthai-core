#pragma once

#include "depthai/common/ADatatypeSharedPtrSerialization.hpp"
#include "depthai/pipeline/datatype/ADatatype.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/utility/ProtoSerializable.hpp"
#include "depthai/utility/Serialization.hpp"

namespace dai {

/**
 * RGBD message. Carries RGB and Depth frames.
 */
class RGBDData : public Buffer, public ProtoSerializable {
   public:
    /**
     * Construct RGBD message.
     */
    RGBDData() = default;

    virtual ~RGBDData();

    std::map<std::string, std::shared_ptr<ADatatype>> frames;
    void setRGBFrame(const std::shared_ptr<ImgFrame>& frame);
    void setDepthFrame(const std::shared_ptr<ImgFrame>& frame);
    std::shared_ptr<ImgFrame> getRGBFrame();
    std::shared_ptr<ImgFrame> getDepthFrame();

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override;
    DatatypeEnum getDatatype() const override {
        return DatatypeEnum::RGBDData;
    }

#ifdef DEPTHAI_ENABLE_PROTOBUF
    /**
     * Serialize message to proto buffer
     *
     * @returns serialized message
     */
    std::vector<std::uint8_t> serializeProto(bool metadataOnly = false) const override;

    /**
     * Serialize schema to proto buffer
     *
     * @returns serialized schema
     */
    ProtoSerializable::SchemaPair serializeSchema() const override;
#endif

    DEPTHAI_SERIALIZE(RGBDData, frames, Buffer::ts, Buffer::tsDevice, Buffer::sequenceNum);
};

}  // namespace dai
