#pragma once

#include "depthai/utility/export.hpp"

// memfd_create wrapper for glibc < 2.27
#if defined(__unix__) && !defined(__APPLE__)
    #if(__GLIBC__ <= 2) && (__GLIBC_MINOR__ < 27)

DEPTHAI_API int memfd_create(const char* name, unsigned int flags);

    #endif
#endif
