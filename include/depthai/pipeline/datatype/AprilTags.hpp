#pragma once

#include <unordered_map>
#include <vector>

#include "depthai-shared/datatype/RawAprilTags.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {

/**
 * AprilTags message.
 */
class AprilTags : public Buffer {
    std::shared_ptr<RawBuffer> serialize() const override;
    RawAprilTags& rawdata;

   public:
    /**
     * Construct AprilTags message.
     */
    AprilTags();
    explicit AprilTags(std::shared_ptr<RawAprilTags> ptr);
    virtual ~AprilTags() = default;

    std::vector<AprilTag>& aprilTags;
};

}  // namespace dai
