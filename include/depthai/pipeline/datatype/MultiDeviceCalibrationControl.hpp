#pragma once

#include <cstdint>
// Keep Serialization before variant.hpp: the latter specializes libnop types
// declared by Serialization.hpp.
// clang-format off
#include <depthai/pipeline/datatype/Buffer.hpp>
#include <depthai/utility/Serialization.hpp>
#include <depthai/common/variant.hpp>
// clang-format on
#include <memory>
#include <utility>
#include <variant>
#include <vector>

namespace dai {

/**
 * @brief One-shot lifecycle commands for the MultiDeviceCalibration node.
 *
 * Configuration belongs to the node and must be set before start. The control
 * message intentionally contains no performance or device-mutation commands.
 */
class MultiDeviceCalibrationControl : public Buffer {
   public:
    struct Commands {
        struct Start {};
        struct Stop {};
        struct Reset {};
    };

    using Command = std::variant<std::monostate, Commands::Start, Commands::Stop, Commands::Reset>;

    Command command{};

    MultiDeviceCalibrationControl() = default;
    explicit MultiDeviceCalibrationControl(Command command) : command(std::move(command)) {}
    ~MultiDeviceCalibrationControl() override;

    [[nodiscard]] static std::shared_ptr<MultiDeviceCalibrationControl> start() {
        return std::make_shared<MultiDeviceCalibrationControl>(Commands::Start{});
    }

    [[nodiscard]] static std::shared_ptr<MultiDeviceCalibrationControl> stop() {
        return std::make_shared<MultiDeviceCalibrationControl>(Commands::Stop{});
    }

    [[nodiscard]] static std::shared_ptr<MultiDeviceCalibrationControl> reset() {
        return std::make_shared<MultiDeviceCalibrationControl>(Commands::Reset{});
    }

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = getDatatype();
    }

    DatatypeEnum getDatatype() const override {
        return DatatypeEnum::MultiDeviceCalibrationControl;
    }
};

}  // namespace dai
