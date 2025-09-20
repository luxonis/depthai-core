#if defined _WIN32 || defined __CYGWIN__
    #define DEPTHAI_HELPER_DLL_IMPORT __declspec(dllimport)
    #define DEPTHAI_HELPER_DLL_EXPORT __declspec(dllexport)
#else
    #if __GNUC__ >= 4
        #define DEPTHAI_HELPER_DLL_IMPORT __attribute__((visibility("default")))
        #define DEPTHAI_HELPER_DLL_EXPORT __attribute__((visibility("default")))
    #else
        #define DEPTHAI_HELPER_DLL_IMPORT
        #define DEPTHAI_HELPER_DLL_EXPORT
    #endif
#endif

#ifdef USE_DEPTHAI_DLL
    #ifdef DEPTHAI_DLL_EXPORT
        #define DEPTHAI_API DEPTHAI_HELPER_DLL_EXPORT
    #else
        #define DEPTHAI_API DEPTHAI_HELPER_DLL_IMPORT
    #endif  // DEPTHAI_DLL_EXPORT
#else       // USE_DEPTHAI_DLL is not defined: this means DEPTHAI is a static lib
    #define DEPTHAI_API
#endif  // DEPTHAI_DLL