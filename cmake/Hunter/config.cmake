hunter_config(
    nlohmann_json_schema_validator
    VERSION "2.1.1"
    URL "https://github.com/luxonis/json-schema-validator/archive/hunter-2.1.1.tar.gz"
    SHA1 "1c410bcf1418155215e6b6b8f31284ff66f0f3b4"
    CMAKE_ARGS
        BUILD_TESTS=OFF
        BUILD_EXAMPLE=OFF
        JSON_VALIDATOR_INSTALL=ON
        HUNTER_ENABLED=ON
        CMAKE_POSITION_INDEPENDENT_CODE=ON
)

hunter_config(
    XLink
    VERSION "luxonis-2020.2.0.1"
    URL  "https://github.com/luxonis/XLink/archive/49528345a0e2f307d01f5eedc1a4ad47615af08b.zip"
    SHA1 "5ae8044555f1772a85ec9100ef5446b9ee2c4a13"
    CMAKE_ARGS
        CMAKE_POSITION_INDEPENDENT_CODE=ON
)

# "https://github.com/luxonis/XLink/archive/test_usb_info.zip"
hunter_config(
    BZip2
    VERSION "1.0.8-p0"
    CMAKE_ARGS
        CMAKE_POSITION_INDEPENDENT_CODE=ON
)

hunter_config(
    Boost
    VERSION "1.72.0-p0"
    CMAKE_ARGS
        CMAKE_POSITION_INDEPENDENT_CODE=ON
)
