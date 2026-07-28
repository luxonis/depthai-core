#pragma once

#include <memory>
#include <string>
#include <variant>

#include "depthai/modelzoo/Zoo.hpp"
#include "depthai/nn_archive/NNArchive.hpp"
#include "depthai/nn_archive/v1/Head.hpp"
#include "depthai/pipeline/ThreadedHostNode.hpp"
#include "depthai/pipeline/datatype/NNData.hpp"

namespace dai {
namespace beta {
namespace node {

/**
 * @brief ImageOutputParser node. Parses the output of image-to-image models (e.g. DnCNN3, zero-dce) where the output is a modified image (denoised, enhanced
 * etc.) into a dai::ImgFrame message.
 *
 * The parser consumes a single output tensor. When the incoming NNData contains exactly one tensor, it is selected automatically; otherwise the output layer
 * name must be configured explicitly or through an NNArchive head. The tensor is read in its stored order; after squeezing a leading batch dimension of 1 it
 * must be a 3D image tensor in CHW or HWC orientation, with the channel dimension equal to 1 (grayscale) or 3 (color). All dimensions are derived from the
 * runtime tensor descriptor. The values are min-max normalized and scaled to the [0, 255] 8-bit range.
 *
 * A grayscale image is emitted as a GRAY8 frame. A color image is emitted as a BGR888p frame when the pipeline's default device platform is RVC2 and as a
 * BGR888i frame otherwise, including in a device-less pipeline. The model output is treated as RGB and converted to BGR unless the BGR-output flag marks it
 * as already BGR.
 *
 * @note This node runs on the host only.
 */
class ImageOutputParser : public NodeCRTP<dai::node::ThreadedHostNode, ImageOutputParser> {
   public:
    constexpr static const char* NAME = "ImageOutputParser";
    using Model = std::variant<NNModelDescription, NNArchive, std::string>;

    /**
     * Input NN results with image tensor data to parse.
     */
    Input input{*this, {"input", DEFAULT_GROUP, DEFAULT_BLOCKING, DEFAULT_QUEUE_SIZE, {{{DatatypeEnum::NNData, true}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    /**
     * Outputs ImgFrame message with the model output image, e.g. a denoised or enhanced image.
     */
    Output out{*this, {"out", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, false}}}}};

    /**
     * @brief Build ImageOutputParser node. Links the supplied output to this node's input and configures the parser from the model's ImageOutputParser head.
     * @param nnInput: Output to link
     * @param model: Neural network model
     */
    std::shared_ptr<ImageOutputParser> build(Node::Output& nnInput, const Model& model);

    /**
     * @brief Build ImageOutputParser node with the specific head from an NNArchive. Useful when the model has multiple heads.
     * @param nnInput: Output to link
     * @param head: Specific head from NNArchive to use for this parser
     */
    std::shared_ptr<ImageOutputParser> build(Node::Output& nnInput, const dai::nn_archive::v1::Head& head);

    /**
     * @brief Set NNArchive for this Node. The archive must contain exactly one ImageOutputParser head; use setNNArchiveHead() to select a specific head from a
     * multi-head archive.
     *
     * @param nnArchive: NNArchive to set
     */
    void setNNArchive(const NNArchive& nnArchive);

    /**
     * @brief Set NNArchive head for this Node. The head must be an ImageOutputParser head with exactly one output layer.
     *
     * @param head: NNArchive head to set
     */
    void setNNArchiveHead(const dai::nn_archive::v1::Head& head);

    /**
     * Sets the name of the model output layer to parse.
     *
     * When left empty, the parser selects the tensor automatically if the incoming NNData contains exactly one tensor and fails otherwise.
     *
     * @param outputLayerName Name of the output layer
     */
    void setOutputLayerName(const std::string& outputLayerName);

    /**
     * Returns the name of the model output layer to parse.
     */
    std::string getOutputLayerName() const;

    /**
     * Sets the flag indicating whether the model output image is in BGR (Blue-Green-Red) channel order.
     *
     * When false (the default), a color model output is treated as RGB and its channels are swapped to BGR before being emitted.
     *
     * @param outputIsBGR True when the model output image is already BGR, defaults to true
     */
    void setBGROutput(bool outputIsBGR = true);

    /**
     * Returns the flag indicating whether the model output image is in BGR (Blue-Green-Red) channel order.
     */
    bool getBGROutput() const;

    void run() override;

   private:
    void setConfig(const dai::NNArchiveVersionedConfig& config);
    void setConfig(const dai::nn_archive::v1::Head& head);
    NNArchive decodeModel(const Model& model);
    NNArchive createNNArchive(NNModelDescription& modelDesc);

    std::string outputLayerName;
    bool outputIsBGR = false;
};

}  // namespace node
}  // namespace beta
}  // namespace dai
