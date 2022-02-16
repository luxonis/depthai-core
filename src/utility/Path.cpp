#if defined(_WIN32) && defined(_MSC_VER)

    #include "utility/Path.hpp"

    #include <codecvt>
    #include <cwchar>
    #include <locale>

namespace dai {
std::wstring Path::convert_utf8_to_wide(const std::string& utf8string) {
    //#pragma warning(suppress : 4996)
    std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>> converter(dai::Path::convert_err, dai::Path::convert_err_wide);
    return converter.from_bytes(utf8string);
}

std::string Path::u8string() const noexcept {
    //#pragma warning(suppress : 4996)
    std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>> converter(dai::Path::convert_err, dai::Path::convert_err_wide);
    return converter.to_bytes(nativePath);
}

std::string Path::string() const noexcept {
    std::mbstate_t state = std::mbstate_t();
    const wchar_t* pNativePath = nativePath.c_str();
    const size_t len = std::wcsrtombs(nullptr, &pNativePath, 0, &state);
    if(len == static_cast<std::size_t>(-1)) {
        return dai::Path::convert_err;
    }
    std::string mbstr(len + 1, 0);  // +1 for terminating null placed by wcsrtombs()
    const size_t retVal = std::wcsrtombs(&mbstr[0], &pNativePath, mbstr.size(), &state);
    mbstr.resize(len);  // remove the extra terminating null
    return mbstr;
}

}  // namespace dai

#endif
