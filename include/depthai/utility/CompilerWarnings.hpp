// Check for GCC or Clang
#pragma once
#if defined(__clang__) || defined(__GNUC__)
    // Helper macro to stringify pragma content correctly
    #define DO_PRAGMA(X) _Pragma(#X)

    // Macro to begin suppressing deprecation warnings
    #define DEPTHAI_BEGIN_SUPPRESS_DEPRECATION_WARNING \
        DO_PRAGMA(GCC diagnostic push)                 \
        DO_PRAGMA(GCC diagnostic ignored "-Wdeprecated-declarations")

    // Macro to end suppressing deprecation warnings
    #define DEPTHAI_END_SUPPRESS_DEPRECATION_WARNING DO_PRAGMA(GCC diagnostic pop)

// Check for MSVC
#elif defined(_MSC_VER)
    // Macro to begin suppressing deprecation warnings (C4996)
    #define DEPTHAI_BEGIN_SUPPRESS_DEPRECATION_WARNING __pragma(warning(push)) __pragma(warning(disable : 4996))

    // Macro to end suppressing deprecation warnings
    #define DEPTHAI_END_SUPPRESS_DEPRECATION_WARNING __pragma(warning(pop))

// Other compilers get placeholder macros (do nothing)
#else
    #define DEPTHAI_BEGIN_SUPPRESS_DEPRECATION_WARNING
    #define DEPTHAI_END_SUPPRESS_DEPRECATION_WARNING
#endif
