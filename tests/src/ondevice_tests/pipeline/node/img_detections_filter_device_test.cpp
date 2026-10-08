#include "../../../onhost_tests/pipeline/node/img_detections_filter_test_cases.hpp"

using namespace filtertest;

TEST_CASE("ImgDetectionsFilter V-1: RVC4 single input defaults to device", "[ImgDetectionsFilter][EX-1]") {
    FilterHarness h({}, {"cam"}, true);
    const auto device = h.pipeline.getDefaultDevice();
    REQUIRE(device != nullptr);
    if(device->getPlatform() != dai::Platform::RVC4) SKIP("V-1 requires RVC4");
    h.pipeline.build();
    REQUIRE_FALSE(h.runOnHost());
}
TEST_CASE("ImgDetectionsFilter V-1b: RVC2 defaults to host", "[ImgDetectionsFilter][EX-1]") {
    FilterHarness h({}, {"cam"}, true);
    const auto device = h.pipeline.getDefaultDevice();
    REQUIRE(device != nullptr);
    if(device->getPlatform() != dai::Platform::RVC2) SKIP("V-1b requires RVC2");
    h.pipeline.build();
    REQUIRE(h.runOnHost());
}
TEST_CASE("ImgDetectionsFilter V-1c: unlinked key does not change device placement", "[ImgDetectionsFilter][EX-1][IN-4]") {
    FilterHarness h({}, {"a"}, true);
    const auto device = h.pipeline.getDefaultDevice();
    REQUIRE(device != nullptr);
    if(device->getPlatform() != dai::Platform::RVC4) SKIP("V-1c requires RVC4");
    h.accessUnlinked("b");
    h.pipeline.build();
    REQUIRE_FALSE(h.runOnHost());
}
TEST_CASE("ImgDetectionsFilter V-2: multiple keys default to host", "[ImgDetectionsFilter][EX-1]") {
    FilterSettings s;
    s.reference = transformation();
    FilterHarness h(s, {"a", "b"}, true);
    const auto device = h.pipeline.getDefaultDevice();
    REQUIRE(device != nullptr);
    if(device->getPlatform() != dai::Platform::RVC4) SKIP("V-2 requires RVC4");
    h.pipeline.build();
    REQUIRE(h.runOnHost());
}
TEST_CASE("ImgDetectionsFilter V-3: device placement rejects multiple keys", "[ImgDetectionsFilter][EX-2][ER-3]") {
    FilterSettings s;
    s.reference = transformation();
    FilterHarness h(s, {"a", "b"}, true);
    const auto device = h.pipeline.getDefaultDevice();
    REQUIRE(device != nullptr);
    if(device->getPlatform() != dai::Platform::RVC4) SKIP("V-3 requires RVC4");
    h.setRunOnHost(false);
    REQUIRE_THROWS(h.pipeline.start());
}
TEST_CASE("ImgDetectionsFilter V-4: explicit host placement", "[ImgDetectionsFilter][EX-1]") {
    FilterHarness h({}, {"cam"}, true);
    const auto device = h.pipeline.getDefaultDevice();
    REQUIRE(device != nullptr);
    if(device->getPlatform() != dai::Platform::RVC4) SKIP("V-4 requires RVC4");
    h.setRunOnHost(true);
    h.pipeline.build();
    REQUIRE(h.runOnHost());
}
TEST_CASE("ImgDetectionsFilter V-5: shared host oracles on RVC4", "[ImgDetectionsFilter][EX-3]") {
    for(const auto* id : {"B-1", "B-3", "B-4", "B-8", "D-1", "E-1", "E-2", "R-1", "R-3"}) {
        DYNAMIC_SECTION(id) {
            runSharedCase(id, true);
        }
    }
}
TEST_CASE("ImgDetectionsFilter V-6: all config options roundtrip through hardware", "[ImgDetectionsFilter][skill-E]") {
    dai::Pipeline pipeline;
    REQUIRE(pipeline.getDefaultDevice() != nullptr);
    auto script = pipeline.create<dai::node::Script>();
    script->setScript("while True:\n    node.outputs['out'].send(node.inputs['in'].get())\n");
    const auto input = script->inputs["in"].createInputQueue();
    const auto output = script->outputs["out"].createOutputQueue();
    const auto config = makeConfig(nondefaultSettings());
    const auto expected = dai::utility::serialize(*config);
    pipeline.start();
    input->send(config);
    bool timedOut = false;
    const auto result = output->get<dai::ImgDetectionsFilterConfig>(std::chrono::seconds(2), timedOut);
    REQUIRE_FALSE(timedOut);
    REQUIRE(result != nullptr);
    REQUIRE(result->getDatatype() == config->getDatatype());
    REQUIRE(dai::utility::serialize(*result) == expected);
}
