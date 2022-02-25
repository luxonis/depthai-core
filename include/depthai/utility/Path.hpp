#pragma once
#if(__cplusplus >= 201703L) || (_MSVC_LANG >= 201703L)
    #include <filesystem>
    #define DAI_NODISCARD [[nodiscard]]
#else
    #define DAI_NODISCARD
#endif
#include <string>

namespace dai {

// TODO C++20 char8_t
// TODO test if caller works when replace "dai::Path" -> "std::filesystem::path"
// TODO test if can `using dai::Path = std::filesystem::path` on C++17 to completely use STL

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

    Path() = default;
    ~Path() = default;
    Path(const Path&) = default;
    Path(Path&&) = default;
    Path& operator=(const Path&) = default;
    Path& operator=(Path&&) = default;

    Path(string_type&& source) noexcept : _nativePath(std::move(source)) {}
    Path(const string_type& source) : _nativePath(source) {}
    Path(const value_type* source) : _nativePath(string_type(source)) {}

#if defined(__cpp_lib_filesystem)
    Path(const std::filesystem::path& source) : _nativePath(source) {}
#endif

#if defined(_WIN32) && defined(_MSC_VER)
   private:
    static std::wstring convert_utf8_to_wide(const std::string& utf8string);

   public:
    Path(const std::string& source) : _nativePath(convert_utf8_to_wide(source)) {}
    Path(const char* source) : _nativePath(convert_utf8_to_wide(std::string(source))) {}
    std::string string() const;
    std::string u8string() const;
#else
    std::string string() const noexcept {
        return _nativePath;
    }
    std::string u8string() const noexcept {
        return _nativePath;
    }
#endif

    // implicitly convert *this to native format string
    operator string_type() const noexcept {
        return _nativePath;
    }

    const string_type& native() const noexcept {
        return _nativePath;
    }

    DAI_NODISCARD bool empty() const noexcept {
        return _nativePath.empty();
    }

   private:
    string_type _nativePath;
};

}  // namespace dai
