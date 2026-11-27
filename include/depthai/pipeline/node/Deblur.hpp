#pragma once
#include "depthai/pipeline/DeviceNode.hpp"
#include "depthai/properties/DeblurProperties.hpp"
namespace dai {
namespace node {
/**
 * Causal spatially varying deblurring on RVC4. Consumes OpticalFlow packets;
 * does not estimate motion itself. Packed GRAY8 only. Restoration is approximate
 * and assumes constant motion during exposure; it is not lossless recovery.
 * See examples/python/Deblur/README.md for synchronization and limitations.
 */
class Deblur : public DeviceNodeCRTP<DeviceNode, Deblur, DeblurProperties> {
   public:
    constexpr static const char* NAME = "Deblur";
    using DeviceNodeCRTP::DeviceNodeCRTP;
    /** Images from OpticalFlow.passthroughInputImage. Nonblocking queue, size four. */
    Input inputImage{*this, {"inputImage", DEFAULT_GROUP, false, 4, {{{DatatypeEnum::ImgFrame, false}}}, false}};
    /** Native OpticalFlow MessageGroup packets. Nonblocking queue, size four. */
    Input inputFlow{*this, {"inputFlow", DEFAULT_GROUP, false, 4, {{{DatatypeEnum::MessageGroup, false}}}, false}};
    /** Restored image or unchanged passthrough with target metadata; never mutates inputs. */
    Output output{*this, {"output", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, false}}}}};
    /** The exact unmodified input corresponding to each output. */
    Output passthroughInputImage{*this, {"passthroughInputImage", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, false}}}}};
    /** GRAY8 effective blend mask [0,255]; zero means the source pixel is preserved. */
    Output outputMask{*this, {"outputMask", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, false}}}}};
    /** Set validated initial configuration. Call before starting the pipeline. */
    Deblur& setConfig(const DeblurProperties& config);
    /** Get a copy of the initial configuration. */
    DeblurProperties getConfig() const;
    /** Validate configuration and RVC4 platform during pipeline construction. */
    void buildInternal() override;
};
}  // namespace node
}  // namespace dai
