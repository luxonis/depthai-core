#pragma once
#if(__cplusplus >= 201703L) || (_MSVC_LANG >= 201703L)
    #include <filesystem>
#endif
#include <string>

namespace dai {

// TODO C++20 char8_t
// TODO test if caller works when replace "dai::Path" -> "std::filesystem::path"
// TODO test if can `using Path = std::filesystem::path` on C++17 to completely use STL

/**
 * @brief accepts utf-8, Windows wchar_t, or std::filesystem::path
 *        and returns the path in a type accepted by fstream()
 *
 */
class Path {
   public:
#if defined(_WIN32) && defined(_MSC_VER)
    using value_type = wchar_t;
#else
    using value_type = char;
#endif
    using string_type = std::basic_string<value_type>;

    string_type nativePath;
    std::wstring convert_utf8_to_wide(const std::string& utf8string);

   public:
    Path() = default;
    ~Path() = default;
    Path(const Path&) = default;
    Path(Path&&) = default;
    Path& operator=(const Path&) = default;
    Path& operator=(Path&&) = default;

    Path(string_type&& source) noexcept : nativePath(std::move(source)) {}
    Path(const string_type& source) : nativePath(source) {}
    Path(const value_type* source) : nativePath(string_type(source)) {}

#if defined(__cpp_lib_filesystem)
    // Path(std::filesystem::path source) : nativePath(source) {}
    Path(const std::filesystem::path& source) : nativePath(source) {}
#endif

#if defined(_WIN32) && defined(_MSC_VER)
    // Path(std::string source) : nativePath(convert_utf8_to_wide(source)) {}
    Path(const std::string& source) : nativePath(convert_utf8_to_wide(source)) {}
    Path(const char* source) : nativePath(convert_utf8_to_wide(std::string(source))) {}
#endif

    // implicitly convert *this to native format string
    operator string_type() const noexcept {
        return nativePath;
    }
};

}  // namespace dai
