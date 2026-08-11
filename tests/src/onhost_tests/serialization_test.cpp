#include <catch2/catch_all.hpp>
#include <filesystem>
#include <fstream>
#include <limits>

// Include depthai library
#include <depthai/depthai.hpp>

TEST_CASE("Roundtrip") {
    dai::Pipeline p;
    auto stereo = p.create<dai::node::StereoDepth>();

    // Create ground truth properties
    stereo->setInputResolution(0xa0a0a0a0, 0xa0a0a0a0);
    stereo->setOutputSize(0x55555555, 0x55555555);
    stereo->setExtendedDisparity(true);
    stereo->properties.numFramesPool = 42;

    // Round trip
    {
        auto ser = dai::utility::serialize(stereo->properties);
        dai::node::StereoDepth::Properties des;
        dai::utility::deserialize(ser, des);
        for(uint8_t b : ser) {
            printf("%02X ", b);
        }
        printf("\n");

        REQUIRE(des.width.value() == (int)0xa0a0a0a0);
        REQUIRE(des.height.value() == (int)0xa0a0a0a0);
        REQUIRE(des.outWidth.value() == (int)0x55555555);
        REQUIRE(des.outHeight.value() == (int)0x55555555);
        REQUIRE(des.numFramesPool == 42);
    }

    // Round trip through pipeline
    {
        auto ser = p.getPipelineSchema().nodes[0].properties;
        dai::node::StereoDepth::Properties des;
        dai::utility::deserialize(ser, des);
        for(uint8_t b : ser) {
            printf("%02X ", b);
        }
        printf("\n");

        REQUIRE(des.width.value() == (int)0xa0a0a0a0);
        REQUIRE(des.height.value() == (int)0xa0a0a0a0);
        REQUIRE(des.outWidth.value() == (int)0x55555555);
        REQUIRE(des.outHeight.value() == (int)0x55555555);
        REQUIRE(des.numFramesPool == 42);
    }
}

TEST_CASE("AssetManager uses the current size of memory-backed assets") {
    dai::AssetManager assetManager;
    auto asset = assetManager.set("asset", std::vector<std::uint8_t>{1, 2});
    asset->data.push_back(3);

    REQUIRE(asset->getSize() == 3);
    REQUIRE(assetManager.getSerializedSize() == 3);
}

TEST_CASE("AssetManager rejects storage beyond 4 GiB") {
    dai::Asset asset("oversized");
    asset.path = "placeholder";
    asset.size = static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max()) + 1;

    dai::AssetManager assetManager;
    assetManager.set(std::move(asset));

    REQUIRE_THROWS_WITH(assetManager.getSerializedSize(), "Asset storage cannot exceed 4 GiB");
}

TEST_CASE("AssetManager preserves the size of path-backed assets when renaming them") {
    dai::Asset asset("source");
    asset.path = "placeholder";
    asset.size = 42;

    dai::AssetManager assetManager;
    auto renamedAsset = assetManager.set("renamed", std::move(asset));

    REQUIRE(renamedAsset->getSize() == 42);
}

TEST_CASE("AssetManager restores storage when a path-backed asset changes") {
    const auto path = std::filesystem::temp_directory_path() / "depthai_asset_manager_serialization_test.bin";
    {
        std::ofstream stream(path, std::ios::binary);
        stream.write("ab", 2);
    }

    dai::AssetManager assetManager;
    assetManager.set("asset", path);
    {
        std::ofstream stream(path, std::ios::binary | std::ios::app);
        stream.write("c", 1);
    }

    dai::AssetsMutable assets;
    std::vector<std::uint8_t> storage{42};
    REQUIRE_THROWS(assetManager.serialize(assets, storage));
    REQUIRE(storage == std::vector<std::uint8_t>{42});

    std::filesystem::remove(path);
}
