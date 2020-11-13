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
    VERSION "luxonis-2020.2-mx_id_compatible"
    URL "https://github.com/luxonis/XLink/archive/04bf4dc9be5da5bd471f6df94dbbe995e425d7b1.tar.gz"
    SHA1 "5aef4de522ae1f0c8a9493158f1bf3a8332cd588"
    CMAKE_ARGS
        CMAKE_POSITION_INDEPENDENT_CODE=ON
)

hunter_config(
    BZip2
    VERSION "1.0.8-p0"
    CMAKE_ARGS
        CMAKE_POSITION_INDEPENDENT_CODE=ON
)