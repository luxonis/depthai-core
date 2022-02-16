#if defined(_WIN32) && defined(_MSC_VER)

    #include "utility/Path.hpp"

    #include <codecvt>
    #include <locale>

namespace dai {

std::wstring Path::convert_utf8_to_wide(const std::string& utf8string) {
    //#pragma warning(suppress : 4996)
    std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>> converter;
    return converter.from_bytes(utf8string);
}

}  // namespace dai

#endif
