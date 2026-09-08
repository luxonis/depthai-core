#pragma once

#include <cstdint>
#include <depthai/common/CameraBoardSocket.hpp>
#include <depthai/device/MultiDeviceCalibrationHandler.hpp>
#include <depthai/pipeline/datatype/Buffer.hpp>
#include <depthai/utility/Serialization.hpp>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace dai {

/**
 * @brief Pure data emitted by a MultiDeviceCalibration node.
 *
 * The handler is a snapshot and is never applied to a device or pipeline by
 * the node. Diagnostics retain the measurements and rejection reason for each
 * candidate edge considered during the run.
 */
struct MultiDeviceCalibrationResult : public Buffer {
    struct EdgeDiagnostic {
        std::string fromDeviceId;
        CameraBoardSocket fromSocket = CameraBoardSocket::AUTO;
        std::string toDeviceId;
        CameraBoardSocket toSocket = CameraBoardSocket::AUTO;
        bool accepted = false;
        double dclConfidence = 0.0;
        double reprojectionError = 0.0;
        double sampsonError = 0.0;
        std::string scaleSource;
        double scaleResidual = 0.0;
        std::string rejectionReason;

        DEPTHAI_SERIALIZE(EdgeDiagnostic,
                          fromDeviceId,
                          fromSocket,
                          toDeviceId,
                          toSocket,
                          accepted,
                          dclConfidence,
                          reprojectionError,
                          sampsonError,
                          scaleSource,
                          scaleResidual,
                          rejectionReason);
    };

    MultiDeviceCalibrationResult() = default;
    explicit MultiDeviceCalibrationResult(std::string information) : info(std::move(information)) {}
    ~MultiDeviceCalibrationResult() override;

    std::optional<MultiDeviceCalibrationHandler> handler;
    bool passed = false;
    bool complete = false;
    double dataConfidence = 0.0;
    std::string info;
    std::vector<EdgeDiagnostic> diagnostics;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override;

    DatatypeEnum getDatatype() const override {
        return DatatypeEnum::MultiDeviceCalibrationResult;
    }

    DEPTHAI_SERIALIZE(MultiDeviceCalibrationResult, handler, passed, complete, dataConfidence, info, diagnostics);
};

}  // namespace dai
