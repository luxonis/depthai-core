#include <catch2/catch_all.hpp>
#include <filesystem>
#include <fstream>
#include <future>
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

TEST_CASE("AssetManager snapshots eagerly loaded path assets") {
    const auto path = std::filesystem::temp_directory_path() / "depthai_asset_manager_eager_path_test.bin";
    {
        std::ofstream stream(path, std::ios::binary);
        stream.write("ab", 2);
    }

    dai::AssetManager assetManager;
    auto asset = assetManager.set("asset", path);
    REQUIRE(asset->data == std::vector<std::uint8_t>{'a', 'b'});
    std::filesystem::remove(path);

    dai::AssetsMutable assets;
    std::vector<std::uint8_t> storage;
    assetManager.serialize(assets, storage);
    REQUIRE(storage == std::vector<std::uint8_t>{'a', 'b'});
}

TEST_CASE("AssetManager rejects storage beyond 4 GiB") {
    dai::Asset asset("oversized");
    asset.setFile("placeholder", static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max()) + 1);

    dai::AssetManager assetManager;
    assetManager.set(std::move(asset));

    REQUIRE_THROWS_WITH(assetManager.getSerializedSize(), "Asset storage cannot exceed 4 GiB");
}

TEST_CASE("AssetManager preserves the size of path-backed assets when renaming them") {
    dai::Asset asset("source");
    asset.setFile("placeholder", 42);

    dai::AssetManager assetManager;
    auto renamedAsset = assetManager.set("renamed", std::move(asset));

    REQUIRE(renamedAsset->getSize() == 42);
}

TEST_CASE("AssetManager materializes path-backed assets through const access") {
    const auto path = std::filesystem::temp_directory_path() / "depthai_asset_manager_const_access_test.bin";
    {
        std::ofstream stream(path, std::ios::binary);
        stream.write("ab", 2);
    }

    dai::AssetManager assetManager;
    assetManager.setLazy("asset", path);

    const auto& constAssetManager = assetManager;
    const auto asset = constAssetManager.get("asset");
    REQUIRE(asset != nullptr);
    REQUIRE(asset->getData() == std::vector<std::uint8_t>{'a', 'b'});

    std::filesystem::remove(path);
}

TEST_CASE("AssetManager materializes path-backed assets once under concurrent access") {
    const auto path = std::filesystem::temp_directory_path() / "depthai_asset_manager_concurrent_access_test.bin";
    {
        std::ofstream stream(path, std::ios::binary);
        stream.write("ab", 2);
    }

    dai::AssetManager assetManager;
    auto asset = assetManager.setLazy("asset", path);
    const auto readData = [asset]() { return asset->getData(); };

    auto first = std::async(std::launch::async, readData);
    auto second = std::async(std::launch::async, readData);
    REQUIRE(first.get() == std::vector<std::uint8_t>{'a', 'b'});
    REQUIRE(second.get() == std::vector<std::uint8_t>{'a', 'b'});

    std::filesystem::remove(path);
}

TEST_CASE("AssetManager serializes materialized path-backed asset data") {
    const auto path = std::filesystem::temp_directory_path() / "depthai_asset_manager_materialized_data_test.bin";
    {
        std::ofstream stream(path, std::ios::binary);
        stream.write("ab", 2);
    }

    dai::AssetManager assetManager;
    auto asset = assetManager.setLazy("asset", path);
    asset->getData()[1] = 'c';
    std::filesystem::remove(path);

    dai::AssetsMutable assets;
    std::vector<std::uint8_t> storage;
    assetManager.serialize(assets, storage);
    REQUIRE(storage == std::vector<std::uint8_t>{'a', 'c'});
}

TEST_CASE("AssetManager resolves path-backed assets when they are registered") {
    const auto root = std::filesystem::temp_directory_path() / "depthai_asset_manager_relative_path_test";
    const auto sourceDirectory = root / "source";
    const auto otherDirectory = root / "other";
    std::filesystem::create_directories(sourceDirectory);
    std::filesystem::create_directories(otherDirectory);
    {
        std::ofstream stream(sourceDirectory / "asset.bin", std::ios::binary);
        stream.write("ab", 2);
    }
    {
        std::ofstream stream(otherDirectory / "asset.bin", std::ios::binary);
        stream.write("cd", 2);
    }

    const auto originalDirectory = std::filesystem::current_path();
    std::filesystem::current_path(sourceDirectory);
    dai::AssetManager assetManager;
    assetManager.setLazy("asset", "asset.bin");
    std::filesystem::current_path(otherDirectory);

    dai::AssetsMutable assets;
    std::vector<std::uint8_t> storage;
    assetManager.serialize(assets, storage);
    std::filesystem::current_path(originalDirectory);

    REQUIRE(storage == std::vector<std::uint8_t>{'a', 'b'});
    std::filesystem::remove_all(root);
}

TEST_CASE("AssetManager restores storage when a path-backed asset changes") {
    const auto path = std::filesystem::temp_directory_path() / "depthai_asset_manager_serialization_test.bin";
    {
        std::ofstream stream(path, std::ios::binary);
        stream.write("ab", 2);
    }

    dai::AssetManager assetManager;
    assetManager.setLazy("asset", path);
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

TEST_CASE("AssetManager rolls back all assets when a later path-backed asset changes") {
    const auto path = std::filesystem::temp_directory_path() / "depthai_asset_manager_transaction_test.bin";
    {
        std::ofstream stream(path, std::ios::binary);
        stream.write("ab", 2);
    }

    dai::AssetManager assetManager;
    assetManager.set("first", std::vector<std::uint8_t>{1, 2});
    assetManager.setLazy("second", path);
    {
        std::ofstream stream(path, std::ios::binary | std::ios::app);
        stream.write("c", 1);
    }

    dai::AssetsMutable assets;
    assets.set("existing", 0, 1, 1);
    std::vector<std::uint8_t> storage{42};
    REQUIRE_THROWS(assetManager.serialize(assets, storage));
    REQUIRE(storage == std::vector<std::uint8_t>{42});
    REQUIRE(assets.has("existing"));
    REQUIRE_FALSE(assets.has("first"));
    REQUIRE_FALSE(assets.has("second"));

    std::filesystem::remove(path);
}
