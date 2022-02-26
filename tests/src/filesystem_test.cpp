#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>

// Include depthai library
#include <cstdio>
#include <depthai/depthai.hpp>
#if(__cplusplus >= 201703L) || (_MSVC_LANG >= 201703L)
    #include <filesystem>
#endif
#include <string>

#if defined(_WIN32) && defined(_MSC_VER)
    #define NATIVETYPE std::wstring
    #define MAKENATIVEx(x) L##x
    #define MAKENATIVE(x) MAKENATIVEx(x)
    #define DELETEFILE _wremove
    #define NATIVELENGTH(x) static_cast<std::wstring>(x).length()
#else
    #define NATIVETYPE std::string
    #define MAKENATIVE(x) x
    #define DELETEFILE std::remove
    #define NATIVELENGTH(x) u8length(static_cast<std::string>(x).c_str())
#endif

#define PATH1 "C:\\dir1\\file1.txt"
#define PATH2 "file2.txt"
#define PATH3 "/dir3/dir33/file3.txt"
#define PATH4 u8"\u00e4\u00eb\u00ef\u00f6\u00fc\u00e1\u00e9\u00ed\u00f3\u00fa\u00df\u00c6\u002e\u010c\u011a\u0141"
#define PATH5 "asdf.nothing"
#define FILETEXT "This is a test\n"
#if defined(_WIN32) && defined(_MSC_VER)
    #define LPATH5 L"asdf.nothing"
#endif

int u8length(const char* str) noexcept {
    int count = 0;
    for(; *str != 0; ++str) count += ((*str & 0xc0) != 0x80);
    return count;
}

TEST_CASE("dai::Path utf-8 and native char set handling") {
    const dai::Path emptyPath;
    const dai::Path::string_type emptyString = emptyPath;
    REQUIRE(emptyString.empty());

    const dai::Path path1(PATH1);
    const NATIVETYPE string1(path1);
    REQUIRE(string1 == MAKENATIVE(PATH1));

    const dai::Path path2(PATH2);
    const NATIVETYPE string2(path2);
    REQUIRE(string2 == MAKENATIVE(PATH2));

    const dai::Path path3(PATH3);
    const NATIVETYPE string3(path3);
    REQUIRE(string3 == MAKENATIVE(PATH3));

    char tmp_name4[L_tmpnam];
    REQUIRE(std::tmpnam(tmp_name4) != nullptr);
    std::string string4(tmp_name4);
    string4 += PATH4;
    const dai::Path path4(string4);
    REQUIRE(u8length(string4.c_str()) == NATIVELENGTH(path4));

    REQUIRE_NOTHROW([&]() {
        std::ofstream file(path4);
        file.write(FILETEXT, sizeof(FILETEXT) - 1);
    }());
    CHECK_NOTHROW([&]() {
        std::ifstream file(path4, std::ios::binary);
        if(!file.is_open() || !file.good() || file.bad()) {
            throw std::runtime_error("file not found or corrupted");
        }
        file.exceptions(std::ifstream::failbit | std::ifstream::badbit);
        const char c1 = file.get();
    }());
    DELETEFILE(static_cast<NATIVETYPE>(path4).c_str());

    auto getBlob = [](const dai::Path& path) -> bool { return !static_cast<NATIVETYPE>(path).empty(); };
    REQUIRE(getBlob(string4));
    REQUIRE(getBlob("mypath/myfile.blob"));
    REQUIRE(getBlob([]() -> std::string { return "mypath/rvalue.blob"; }()));
#if defined(_WIN32) && defined(_MSC_VER)
    REQUIRE(getBlob(L"mywidepath/myfile.blob"));
    std::wstring move2ndTry(L"move my data on the second try");
    REQUIRE(getBlob(move2ndTry));
    REQUIRE(getBlob(std::move(move2ndTry)));

    REQUIRE_THROWS_AS(path4.string(), std::range_error);
#else
    REQUIRE_NOTHROW(path4.string());
#endif

    REQUIRE(path3.string().length() == (sizeof(PATH3) - 1));
    REQUIRE(path3.u8string().length() == (sizeof(PATH3) - 1));
    REQUIRE(path4.u8string() == string4);

    auto wrapper = [&getBlob](const dai::Path& path) -> bool { return getBlob(path); };
    REQUIRE(wrapper("pass dai::Path across functions"));

    // test with std::filesystem
#if defined(__cpp_lib_filesystem)
    const std::filesystem::path fspath1(PATH1);
    const std::filesystem::path fspath2(PATH2);
    const std::filesystem::path fspath3(PATH3);
    const std::filesystem::path fspath4(PATH4);
    REQUIRE(getBlob(fspath1));
    REQUIRE(getBlob(fspath2));
    REQUIRE(getBlob(fspath3));
    REQUIRE(getBlob(fspath4));
#endif
}

TEST_CASE("dai::Path with NN blobs") {
    char osTmpPathname[L_tmpnam];
    REQUIRE(std::tmpnam(osTmpPathname) != nullptr);
    std::string strPath(osTmpPathname);
    strPath += PATH4;
    const dai::Path daiPath(strPath);

    dai::Pipeline pipeline;
    auto nn = pipeline.create<dai::node::NeuralNetwork>();

    // attempt to use a non-existing blob at a utf-8 path
#if defined(_WIN32) && defined(_MSC_VER)
    REQUIRE_THROWS_WITH(nn->setBlobPath(PATH4), Catch::Matchers::Contains("Cannot load blob") && Catch::Matchers::Contains("not convertible"));
    REQUIRE_THROWS_WITH(nn->setBlobPath(strPath), Catch::Matchers::Contains("Cannot load blob") && Catch::Matchers::Contains("not convertible"));
    REQUIRE_THROWS_WITH(nn->setBlobPath(daiPath), Catch::Matchers::Contains("Cannot load blob") && Catch::Matchers::Contains("not convertible"));
#else
    REQUIRE_THROWS_WITH(nn->setBlobPath(PATH4), Catch::Matchers::Contains("Cannot load blob") && Catch::Matchers::Contains(PATH4));
    REQUIRE_THROWS_WITH(nn->setBlobPath(strPath), Catch::Matchers::Contains("Cannot load blob") && Catch::Matchers::Contains(strPath));
    REQUIRE_THROWS_WITH(nn->setBlobPath(daiPath), Catch::Matchers::Contains("Cannot load blob") && Catch::Matchers::Contains(daiPath.u8string()));
#endif

    // use blob at known test path
    const std::string blobPath(BLOB_PATH);
    const dai::Path diaPath2(BLOB_PATH);
    const dai::Path diaPath3(blobPath);
    const dai::Path diaPath4(diaPath2);
    REQUIRE_NOTHROW(nn->setBlobPath(BLOB_PATH));
    nn->getAssetManager().remove("__blob");
    REQUIRE_NOTHROW(nn->setBlobPath(blobPath));
    nn->getAssetManager().remove("__blob");
    REQUIRE_NOTHROW(nn->setBlobPath(diaPath2));
    nn->getAssetManager().remove("__blob");
    REQUIRE_NOTHROW(nn->setBlobPath(diaPath3));
    nn->getAssetManager().remove("__blob");
    REQUIRE_NOTHROW(nn->setBlobPath(diaPath4));
    nn->getAssetManager().remove("__blob");

    // use blob at known test path with std::filesystem
#if defined(__cpp_lib_filesystem)
    std::filesystem::path stdPath(BLOB_PATH);
    REQUIRE_NOTHROW(nn->setBlobPath(stdPath));
    nn->getAssetManager().remove("__blob");
#endif
}

TEST_CASE("dai::Path with Device") {
    const char badfile[] = PATH5;
    const std::string strBadfile(&badfile[0]);
    const dai::Path diaBadFile(PATH5);
    dai::Pipeline pipeline;
    auto nn = pipeline.create<dai::node::NeuralNetwork>();
    REQUIRE_NOTHROW(nn->setBlobPath(BLOB_PATH));
    REQUIRE_THROWS_WITH(dai::Device(pipeline, PATH5), Catch::Matchers::Contains(PATH5));
    REQUIRE_THROWS_WITH(dai::Device(pipeline, &badfile[0]), Catch::Matchers::Contains(PATH5));
    REQUIRE_THROWS_WITH(dai::Device(pipeline, strBadfile), Catch::Matchers::Contains(PATH5));
    REQUIRE_THROWS_WITH(dai::Device(pipeline, diaBadFile), Catch::Matchers::Contains(PATH5));

#if defined(_WIN32) && defined(_MSC_VER)
    const wchar_t wideBadfile[] = LPATH5;
    const std::wstring wstrBadfile(LPATH5);
    const dai::Path diaFileFromWide(LPATH5);
    REQUIRE_THROWS_WITH(dai::Device(pipeline, LPATH5), Catch::Matchers::Contains(PATH5));
    REQUIRE_THROWS_WITH(dai::Device(pipeline, &wideBadfile[0]), Catch::Matchers::Contains(PATH5));
    REQUIRE_THROWS_WITH(dai::Device(pipeline, wstrBadfile), Catch::Matchers::Contains(PATH5));
    REQUIRE_THROWS_WITH(dai::Device(pipeline, diaFileFromWide), Catch::Matchers::Contains(PATH5));
#endif

#if defined(__cpp_lib_filesystem)
    std::filesystem::path stdBadfile(PATH5);
    REQUIRE_THROWS_WITH(dai::Device(pipeline, stdBadfile), Catch::Matchers::Contains(PATH5));
#endif

    dai::Device d(pipeline);
}

TEST_CASE("dai::Path with CalibrationHandler") {
    char tmpFilename[L_tmpnam];
    REQUIRE(std::tmpnam(tmpFilename) != nullptr);
    std::string strFilename(tmpFilename);
    strFilename += PATH4;
    dai::Path daiFilename(strFilename);

    CHECK_NOTHROW([&]() {
        dai::Device device;
        dai::CalibrationHandler deviceCalib = device.readCalibration();
        deviceCalib.eepromToJsonFile(strFilename);
        std::ifstream file(daiFilename, std::ios::binary);
        if(!file.is_open() || !file.good() || file.bad()) {
            throw std::runtime_error("calibration file not found or corrupted");
        }
        file.exceptions(std::ifstream::failbit | std::ifstream::badbit);
        const char c1 = file.get();
    }());
    DELETEFILE(static_cast<NATIVETYPE>(daiFilename).c_str());
}

TEST_CASE("dai::Path with DeviceBootloader") {
    const char badfile[] = PATH4;
    const std::string strBadfile(&badfile[0]);
    const dai::Path diaBadfile(PATH4);
#if defined(_WIN32) && defined(_MSC_VER)
    const wchar_t wideBadfile[] = LPATH5;
    const std::wstring wstrBadfile(LPATH5);
    const dai::Path diaBadWide(LPATH5);
#endif
#if defined(__cpp_lib_filesystem)
    auto stdBadpath = std::filesystem::u8path(PATH4);
#endif

    bool found = false;
    dai::DeviceInfo deviceInfo;
    std::tie(found, deviceInfo) = dai::DeviceBootloader::getFirstAvailableDevice();
    if(found) {
        REQUIRE_NOTHROW([&]() {
            dai::DeviceBootloader bl(deviceInfo);
            auto currentBlType = bl.getType();
        }());
        REQUIRE_NOTHROW([&]() {
            dai::DeviceBootloader bl(deviceInfo, false);
            auto currentBlType = bl.getType();
        }());
        REQUIRE_THROWS_WITH(
            [&]() {
                dai::DeviceBootloader bl(deviceInfo, &badfile[0]);
                auto currentBlType = bl.getType();
            }(),
            Catch::Matchers::Contains("doesn't exist"));
        REQUIRE_THROWS_WITH(
            [&]() {
                dai::DeviceBootloader bl(deviceInfo, strBadfile);
                auto currentBlType = bl.getType();
            }(),
            Catch::Matchers::Contains("doesn't exist"));
        REQUIRE_THROWS_WITH(
            [&]() {
                dai::DeviceBootloader bl(deviceInfo, diaBadfile);
                auto currentBlType = bl.getType();
            }(),
            Catch::Matchers::Contains("doesn't exist"));
#if defined(_WIN32) && defined(_MSC_VER)
        REQUIRE_THROWS_WITH(
            [&]() {
                dai::DeviceBootloader bl(deviceInfo, &wideBadfile[0]);
                auto currentBlType = bl.getType();
            }(),
            Catch::Matchers::Contains("doesn't exist"));
        REQUIRE_THROWS_WITH(
            [&]() {
                dai::DeviceBootloader bl(deviceInfo, wstrBadfile);
                auto currentBlType = bl.getType();
            }(),
            Catch::Matchers::Contains("doesn't exist"));
        REQUIRE_THROWS_WITH(
            [&]() {
                dai::DeviceBootloader bl(deviceInfo, diaBadWide);
                auto currentBlType = bl.getType();
            }(),
            Catch::Matchers::Contains("doesn't exist"));
#endif
#if defined(__cpp_lib_filesystem)
        REQUIRE_THROWS_WITH(
            [&]() {
                dai::DeviceBootloader bl(deviceInfo, stdBadpath);
                auto currentBlType = bl.getType();
            }(),
            Catch::Matchers::Contains("doesn't exist"));
#endif
    } else {
        std::cout << "No devices found" << std::endl;
    }
}

TEST_CASE("dai::Path with AssetManager, StereoDepth") {
    char tmp_name4[L_tmpnam];
    REQUIRE(std::tmpnam(tmp_name4) != nullptr);
    std::string string4(tmp_name4);
    string4 += PATH4;
    const dai::Path path4(string4);
    REQUIRE_NOTHROW([&]() {
        std::ofstream file(path4);
        file.write(FILETEXT, sizeof(FILETEXT) - 1);
    }());

    dai::Pipeline pipeline;
    CHECK_NOTHROW(pipeline.setCameraTuningBlobPath(string4.c_str()));
    pipeline.getAssetManager().remove("camTuning");
    CHECK_NOTHROW(pipeline.setCameraTuningBlobPath(string4));
    pipeline.getAssetManager().remove("camTuning");
    CHECK_NOTHROW(pipeline.setCameraTuningBlobPath(path4));
    pipeline.getAssetManager().remove("camTuning");

    auto depth = pipeline.create<dai::node::StereoDepth>();
    CHECK_NOTHROW(depth->loadMeshFiles(string4.c_str(), string4.c_str()));
    depth->getAssetManager().remove("meshLeft");
    depth->getAssetManager().remove("meshRight");
    CHECK_NOTHROW(depth->loadMeshFiles(string4, string4));
    depth->getAssetManager().remove("meshLeft");
    depth->getAssetManager().remove("meshRight");
    CHECK_NOTHROW(depth->loadMeshFiles(path4, path4));
    depth->getAssetManager().remove("meshLeft");
    depth->getAssetManager().remove("meshRight");

#if defined(_WIN32) && defined(_MSC_VER)
    const std::wstring widePath4 = path4.native();
    CHECK_NOTHROW(pipeline.setCameraTuningBlobPath(widePath4.c_str()));
    pipeline.getAssetManager().remove("camTuning");
    CHECK_NOTHROW(pipeline.setCameraTuningBlobPath(widePath4));
    pipeline.getAssetManager().remove("camTuning");

    CHECK_NOTHROW(depth->loadMeshFiles(widePath4.c_str(), widePath4.c_str()));
    depth->getAssetManager().remove("meshLeft");
    depth->getAssetManager().remove("meshRight");
    CHECK_NOTHROW(depth->loadMeshFiles(widePath4, widePath4));
    depth->getAssetManager().remove("meshLeft");
    depth->getAssetManager().remove("meshRight");
#endif

#if defined(__cpp_lib_filesystem)
    auto stdPath4 = std::filesystem::u8path(string4);
    CHECK_NOTHROW(pipeline.setCameraTuningBlobPath(stdPath4));
    CHECK_NOTHROW(depth->loadMeshFiles(stdPath4, stdPath4));
#endif

    DELETEFILE(static_cast<NATIVETYPE>(path4).c_str());
}

TEST_CASE("dai::Path with Script") {
    char tmp_name4[L_tmpnam];
    REQUIRE(std::tmpnam(tmp_name4) != nullptr);
    std::string string4(tmp_name4);
    string4 += PATH4;
    const dai::Path path4(string4);
    REQUIRE_NOTHROW([&]() {
        std::ofstream file(path4);
        file.write(FILETEXT, sizeof(FILETEXT) - 1);
    }());

    dai::Pipeline pipeline;
    auto script = pipeline.create<dai::node::Script>();

    CHECK_NOTHROW(script->setScriptPath(path4));
    CHECK(script->getScriptPath().u8string() == string4);
    CHECK(script->getScriptName() == string4);
    script->getAssetManager().remove("__script");

    CHECK_NOTHROW(script->setScript("<s>nothing</s>"));
    CHECK(script->getScriptPath().empty());
    CHECK(script->getScriptName() == "<script>");
    script->getAssetManager().remove("__script");

    CHECK_NOTHROW(script->setScript("<s>nothing</s>", "myname"));
    CHECK(script->getScriptPath().empty());
    CHECK(script->getScriptName() == "myname");
    script->getAssetManager().remove("__script");

#if defined(_WIN32) && defined(_MSC_VER)
    const std::wstring widePath4 = path4.native();
    CHECK_NOTHROW(script->setScriptPath(widePath4));
    CHECK(script->getScriptPath().native() == widePath4);
    CHECK(script->getScriptName() == string4);
    script->getAssetManager().remove("__script");
#endif

#if defined(__cpp_lib_filesystem)
    auto stdPath4 = std::filesystem::u8path(string4);
    CHECK_NOTHROW(script->setScriptPath(stdPath4));
    CHECK(script->getScriptPath().u8string() == stdPath4.u8string());
    CHECK(script->getScriptName() == string4);
    script->getAssetManager().remove("__script");
#endif

    DELETEFILE(static_cast<NATIVETYPE>(path4).c_str());
}
