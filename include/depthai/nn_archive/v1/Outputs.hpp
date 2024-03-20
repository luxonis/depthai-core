#pragma once

#include <vector>
#include <string>
#include <optional>

namespace dai {
namespace nn_archive {
namespace v1 {
    /**
     * A configuration specifying which output names from the `outputs` block of the archive are
     * fed into the head.
     *
     * Parent class for all outputs.
     *
     * Represents outputs of a basic YOLO object detection model.
     *
     * @type yolo_outputs: C{List[str]}
     * @ivar yolo_outputs: A list of output names for each of the different YOLO grid
     * sizes.
     *
     * Represents outputs of a MobileNet SSD object detection model.
     *
     * @type boxes: str
     * @ivar boxes: Output name corresponding to predicted bounding box coordinates.
     * @type scores: str
     * @ivar scores: Output name corresponding to predicted bounding box confidence scores.
     *
     * Represents outputs of a basic YOLO object detection model.
     *
     * @type yolo_outputs: C{List[str]}
     * @ivar yolo_outputs: A list of output names for each of the different YOLO grid
     * sizes.
     * @type mask_outputs: C{List[str]}
     * @ivar mask_outputs: A list of output names for each mask output.
     * @type protos: str
     * @ivar protos: Output name for the protos.
     */


    /**
     * A configuration specifying which output names from the `outputs` block of the archive are
     * fed into the head.
     *
     * Parent class for all outputs.
     *
     * Represents outputs of a basic YOLO object detection model.
     *
     * @type yolo_outputs: C{List[str]}
     * @ivar yolo_outputs: A list of output names for each of the different YOLO grid
     * sizes.
     *
     * Represents outputs of a MobileNet SSD object detection model.
     *
     * @type boxes: str
     * @ivar boxes: Output name corresponding to predicted bounding box coordinates.
     * @type scores: str
     * @ivar scores: Output name corresponding to predicted bounding box confidence scores.
     *
     * Represents outputs of a basic YOLO object detection model.
     *
     * @type yolo_outputs: C{List[str]}
     * @ivar yolo_outputs: A list of output names for each of the different YOLO grid
     * sizes.
     * @type mask_outputs: C{List[str]}
     * @ivar mask_outputs: A list of output names for each mask output.
     * @type protos: str
     * @ivar protos: Output name for the protos.
     */
    struct Outputs {
        /**
         * Name of the output with predictions.
         */
        std::optional<std::string> predictions;
        /**
         * A list of output names for each of the different YOLO grid sizes.
         */
        std::optional<std::vector<std::string>> yoloOutputs;
        /**
         * Output name corresponding to predicted bounding box coordinates.
         */
        std::optional<std::string> boxes;
        /**
         * Output name corresponding to predicted bounding box confidence scores.
         */
        std::optional<std::string> scores;
        /**
         * A list of output names for each mask output.
         */
        std::optional<std::vector<std::string>> maskOutputs;
        /**
         * Output name for the protos.
         */
        std::optional<std::string> protos;
    };
}
}
}
