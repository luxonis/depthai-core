#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>

// Include depthai library
#include <cstdio>
#include <depthai/depthai.hpp>
#if(__cplusplus >= 201703L) || (_MSVC_LANG >= 201703L)
    #include <filesystem>
#endif
#include <string>

#include "depthai/utility/Path.hpp"

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
#define FILETEXT "This is a test\n"

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
    REQUIRE_NOTHROW([&]() {
        std::ifstream file(path4, std::ios::binary);
        if(!file.is_open() || !file.good() || file.bad()) {
            throw std::runtime_error("file not found or corrupted");
        }
        file.exceptions(std::ifstream::failbit | std::ifstream::badbit);
        const char c1 = file.get();
    }());
    REQUIRE(0 == DELETEFILE(static_cast<NATIVETYPE>(path4).c_str()));

    auto getBlob = [](const dai::Path& path) -> bool { return !static_cast<NATIVETYPE>(path).empty(); };
    REQUIRE(getBlob(string4));
    REQUIRE(getBlob("mypath/myfile.blob"));
    REQUIRE(getBlob([]() -> std::string { return "mypath/rvalue.blob"; }()));
#if defined(_WIN32) && defined(_MSC_VER)
    REQUIRE(getBlob(L"mywidepath/myfile.blob"));
    std::wstring move2ndTry(L"move my data on the second try");
    REQUIRE(getBlob(move2ndTry));
    REQUIRE(getBlob(std::move(move2ndTry)));
#endif

    auto wrapper = [&getBlob](const dai::Path& path) -> bool { return getBlob(path); };
    REQUIRE(wrapper("pass dai::Path across functions"));

    REQUIRE(path3.string().length() == (sizeof(PATH3) - 1));
    REQUIRE(path3.u8string().length() == (sizeof(PATH3) - 1));
    REQUIRE_THROWS_AS(path4.string() == dai::Path::convert_err, std::range_error);
    REQUIRE(path4.u8string() == string4);

#if defined(__cpp_lib_filesystem)
    const std::filesystem::path fspath1(PATH1);
    const std::filesystem::path fspath2(PATH2);
    const std::filesystem::path fspath3(PATH3);
    REQUIRE(getBlob(fspath1));
    REQUIRE(getBlob(fspath2));
    REQUIRE(getBlob(fspath3));
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
    REQUIRE_THROWS_WITH(nn->setBlobPath(PATH4), Catch::Matchers::Contains("Cannot load blob") && Catch::Matchers::Contains("not convertible"));
    REQUIRE_THROWS_WITH(nn->setBlobPath(strPath), Catch::Matchers::Contains("Cannot load blob") && Catch::Matchers::Contains("not convertible"));
    REQUIRE_THROWS_WITH(nn->setBlobPath(daiPath), Catch::Matchers::Contains("Cannot load blob") && Catch::Matchers::Contains("not convertible"));

    // use blob at known test path
    REQUIRE_NOTHROW(nn->setBlobPath(BLOB_PATH));
}
