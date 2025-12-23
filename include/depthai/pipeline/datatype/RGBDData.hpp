#pragma once

#include <optional>
#include <variant>

#include "depthai/common/ADatatypeSharedPtrSerialization.hpp"
#include "depthai/common/optional.hpp"
#include "depthai/common/variant.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/pipeline/datatype/EncodedFrame.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/utility/ProtoSerializable.hpp"
#include "depthai/utility/Serialization.hpp"

namespace dai {

/**
 * RGBD message. Carries RGB and Depth frames.
 * Frames can be either of type ImgFrame or EncodedFrame.
 */
class RGBDData : public Buffer, public ProtoSerializable {
   public:
    using FrameVariant = std::variant<std::shared_ptr<ImgFrame>, std::shared_ptr<EncodedFrame>>;

    /**
     * Construct RGBD message.
     */
    RGBDData() = default;

    virtual ~RGBDData();

    // Setters
    void setRGBFrame(const FrameVariant& frame);
    void setDepthFrame(const FrameVariant& frame);

    // Getters
    std::optional<FrameVariant> getRGBFrame() const;
    std::optional<FrameVariant> getDepthFrame() const;

   private:
    std::optional<std::shared_ptr<ADatatype>> colorFrame;  // ImgFrame or EncodedFrame are both ADatatype
    std::optional<std::shared_ptr<ADatatype>> depthFrame;  // ImgFrame or EncodedFrame are both ADatatype

   public:
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

    DEPTHAI_SERIALIZE(RGBDData, colorFrame, depthFrame, Buffer::ts, Buffer::tsDevice, Buffer::sequenceNum);
};

}  // namespace dai
