#pragma once

#if defined(__unix__) && !defined(__APPLE__)
    #if(__GLIBC__ <= 2) && (__GLIBC_MINOR__ < 27)

int memfd_create(const char* name, unsigned int flags);

    #endif
#endif
