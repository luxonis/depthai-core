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
    URL "https://github.com/luxonis/XLink/archive/f2146735ddb694d1640406694592f9187140731c.tar.gz"
    SHA1 "da1ce78e8f34e6dae4fe99545671b70061ccc9ab"
    CMAKE_ARGS
        CMAKE_POSITION_INDEPENDENT_CODE=ON
)

hunter_config(
    BZip2
    VERSION "1.0.8-p0"
    CMAKE_ARGS
        CMAKE_POSITION_INDEPENDENT_CODE=ON
)