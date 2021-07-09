#pragma once

#include <unordered_map>
#include <vector>

#include "depthai-shared/datatype/RawAprilTags.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {

/**
 * AprilTagData message.
 */
class AprilTagData : public Buffer {
    std::shared_ptr<RawBuffer> serialize() const override;
    RawAprilTags& rawdata;

   public:
    /**
     * Construct AprilTagData message.
     */
    AprilTagData();
    explicit AprilTagData(std::shared_ptr<RawAprilTags> ptr);
    virtual ~AprilTagData() = default;

    std::vector<AprilTags>& aprilTags;
};

}  // namespace dai
