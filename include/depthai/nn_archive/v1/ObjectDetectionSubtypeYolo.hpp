#pragma once

#include <optional>

namespace dai {
namespace nn_archive {
namespace v1 {
/**
 * YOLO family decoding subtype (e.g. v5, v6, v7 etc.).
 *
 * Object detection decoding subtypes for YOLO networks.
 *
 * Subtype members have exactly the same decoding.
 */

/**
 * YOLO family decoding subtype (e.g. v5, v6, v7 etc.).
 *
 * Object detection decoding subtypes for YOLO networks.
 *
 * Subtype members have exactly the same decoding.
 */
enum class ObjectDetectionSubtypeYolo : int { YOLOV5, YOLOV6, YOLOV7, YOLOV8 };
}  // namespace v1
}  // namespace nn_archive
}  // namespace dai
