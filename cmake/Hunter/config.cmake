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
    VERSION "luxonis-2020.2"
    URL "https://github.com/luxonis/XLink/archive/1717591c8bfb078a9a6ed30bca9c82273296e86f.tar.gz"
    SHA1 "06419E6F806B9A3DF64E4929E1262497C172CACE"
    CMAKE_ARGS
        CMAKE_POSITION_INDEPENDENT_CODE=ON
)

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