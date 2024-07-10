#pragma once

#include <optional>

namespace dai {
namespace nn_archive {
namespace v1 {
    /**
     * YOLO family decoding subtype (e.g. yolov5, yolov6, yolov7 etc.).
     *
     * Object detection decoding subtypes for YOLO networks.
     *
     * Subtype members have exactly the same decoding.
     */


    /**
     * YOLO family decoding subtype (e.g. yolov5, yolov6, yolov7 etc.).
     *
     * Object detection decoding subtypes for YOLO networks.
     *
     * Subtype members have exactly the same decoding.
     */
    enum class ObjectDetectionSubtypeYolo : int { YOLOV10, YOLOV5, YOLOV6, YOLOV6_R2, YOLOV7, YOLOV8 };
}
}
}
