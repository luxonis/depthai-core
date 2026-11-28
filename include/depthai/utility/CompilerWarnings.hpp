#pragma once
#if defined(__clang__) || defined(__GNUC__)
    #define DO_PRAGMA(X) _Pragma(#X)

    #define DEPTHAI_BEGIN_SUPPRESS_DEPRECATION_WARNING \
        DO_PRAGMA(GCC diagnostic push)                 \
        DO_PRAGMA(GCC diagnostic ignored "-Wdeprecated-declarations")

    #define DEPTHAI_END_SUPPRESS_DEPRECATION_WARNING DO_PRAGMA(GCC diagnostic pop)

#elif defined(_MSC_VER)
    #define DEPTHAI_BEGIN_SUPPRESS_DEPRECATION_WARNING __pragma(warning(push)) __pragma(warning(disable : 4996))

    #define DEPTHAI_END_SUPPRESS_DEPRECATION_WARNING __pragma(warning(pop))

#else
    #define DEPTHAI_BEGIN_SUPPRESS_DEPRECATION_WARNING
    #define DEPTHAI_END_SUPPRESS_DEPRECATION_WARNING
#endif
