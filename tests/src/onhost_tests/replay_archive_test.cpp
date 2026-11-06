#include <catch2/catch_all.hpp>
#include <fstream>

#include "../ReplayArchiveFixture.hpp"
#include "utility/ReplayArchive.hpp"

TEST_CASE("Replay archive extraction handles directory entries and compression", "[archive]") {
    const auto extension = GENERATE(".tar", ".tar.gz", ".tgz", ".tar.xz");
    replaytest::Directory temporary;
    const auto archive = temporary.path / (std::string("frames") + extension);
    const auto destination = temporary.path / "extracted";
    replaytest::writeArchive(archive, {{"./", {}, true}, {"./frames/", {}, true}, {"./frames/cam_2.jpg", {1, 2, 3, 4}}});
    const auto listing = dai::replay::extractReplayArchive(archive, destination);
    REQUIRE(listing.entryBytes == 4);
    REQUIRE(listing.extractedRoot == destination);
    REQUIRE(dai::replay::selectReplayArchiveSource(destination) == destination / "frames");
    std::ifstream input(destination / "frames" / "cam_2.jpg", std::ios::binary);
    const std::vector<std::uint8_t> bytes((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
    REQUIRE(bytes == std::vector<std::uint8_t>{1, 2, 3, 4});
}

TEST_CASE("Replay archive source selection distinguishes frame directories and videos", "[archive]") {
    replaytest::Directory temporary;
    const auto directory = temporary.path / "frames";
    std::filesystem::create_directories(directory);
    std::ofstream(directory / "clip.mp4") << "video";
    std::ofstream(directory / "clip.mcap") << "metadata";
    REQUIRE(dai::replay::selectReplayArchiveSource(temporary.path) == directory / "clip.mp4");
    std::filesystem::remove(directory / "clip.mp4");
    std::filesystem::remove(directory / "clip.mcap");
    std::ofstream(directory / "cam_0.png") << "image";
    REQUIRE(dai::replay::selectReplayArchiveSource(temporary.path) == directory);
    std::ofstream(directory / "other.mp4") << "video";
    REQUIRE_THROWS(dai::replay::selectReplayArchiveSource(temporary.path));
}

TEST_CASE("Replay archive extraction rejects escaping paths and links", "[archive]") {
    const auto name = GENERATE("../escape.jpg", "/absolute.jpg", "frames/../../escape.jpg");
    replaytest::Directory temporary;
    const auto archive = temporary.path / "unsafe.tar";
    replaytest::writeArchive(archive, {{name, {1}}});
    REQUIRE_THROWS(dai::replay::extractReplayArchive(archive, temporary.path / "out"));
    replaytest::writeArchive(archive, {{"frames/link", {}, false, "../../escape"}});
    REQUIRE_THROWS(dai::replay::extractReplayArchive(archive, temporary.path / "out"));
}
