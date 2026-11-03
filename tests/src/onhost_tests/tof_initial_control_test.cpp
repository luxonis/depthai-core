#include <catch2/catch_all.hpp>
#include <depthai/depthai.hpp>
#include <string>
#include <utility>
#include <vector>

namespace {

using MiscControls = std::vector<std::pair<std::string, std::string>>;

}  // namespace

TEST_CASE("CameraControl frame sync helpers map to sensor misc controls") {
    dai::CameraControl ctrl;
    ctrl.setFrameSyncId(3).setFrameSyncDelayUs(250);
    REQUIRE(ctrl.miscControls == MiscControls{{"frame-sync-id", "3"}, {"frame-sync-delay-us", "250"}});

    // Same as the generic misc control path used with DepthAI v2
    dai::CameraControl viaMisc;
    viaMisc.setMisc("frame-sync-id", 3);
    viaMisc.setMisc("frame-sync-delay-us", 250);
    REQUIRE(viaMisc.miscControls == ctrl.miscControls);
}

TEST_CASE("ToFProperties initialControl survives serialization") {
    dai::ToFProperties props;
    props.fps = 10.0f;
    props.numFramesPoolRaw = 5;
    props.initialControl.setFrameSyncId(2);

    const auto serialized = dai::utility::serialize(props);
    REQUIRE_FALSE(serialized.empty());

    dai::ToFProperties out;
    dai::utility::deserialize(serialized, out);
    REQUIRE(out.fps == Catch::Approx(10.0f));
    REQUIRE(out.numFramesPoolRaw == 5);
    REQUIRE(out.initialControl.miscControls == MiscControls{{"frame-sync-id", "2"}});
}

TEST_CASE("ToFProperties initialControl is appended after the existing fields") {
    // Wire-format compatibility: the new field must be serialized after the previously last field
    // (numFramesPoolRaw), so older firmware/hosts still parse the existing fields at the same place.
    auto firstDiff = [](const std::vector<std::uint8_t>& a, const std::vector<std::uint8_t>& b) {
        size_t i = 0;
        while(i < a.size() && i < b.size() && a[i] == b[i]) i++;
        return i;
    };
    const dai::ToFProperties base{};
    dai::ToFProperties changedPoolRaw;
    changedPoolRaw.numFramesPoolRaw = base.numFramesPoolRaw + 1;
    dai::ToFProperties changedControl;
    changedControl.initialControl.setFrameSyncId(1);

    const auto sBase = dai::utility::serialize(base);
    const auto diffPoolRaw = firstDiff(sBase, dai::utility::serialize(changedPoolRaw));
    const auto diffControl = firstDiff(sBase, dai::utility::serialize(changedControl));
    REQUIRE(diffPoolRaw < sBase.size());
    REQUIRE(diffPoolRaw < diffControl);
}
