hunter_config(
    nlohmann_json
    VERSION "3.9.1"
    URL "https://github.com/nlohmann/json/archive/v3.9.1.tar.gz"
    SHA1 "f8a20a7e19227906d77de0ede97468fbcfea03e7"
    CMAKE_ARGS
        CMAKE_POSITION_INDEPENDENT_CODE=ON
)

hunter_config(
    XLink
    VERSION "luxonis-2020.2-bootloader"
    URL "https://github.com/luxonis/XLink/archive/d6dd3b0a5a8d8e717892970bd24c41f3e7080811.tar.gz"
    SHA1 "134e8e699636ec162a520d2b7e24f7cdfc8c8560"
    CMAKE_ARGS
        CMAKE_POSITION_INDEPENDENT_CODE=ON
)

hunter_config(
    BZip2
    VERSION "1.0.8-p0"
    CMAKE_ARGS
        CMAKE_POSITION_INDEPENDENT_CODE=ON
)