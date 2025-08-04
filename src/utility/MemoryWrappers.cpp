#include "depthai/utility/MemoryWrappers.hpp"

// memfd_create wrapper for glibc < 2.27
#if defined(__unix__) && !defined(__APPLE__)
    #if (__GLIBC__ <= 2) && (__GLIBC_MINOR__ < 27)
        #include <sys/syscall.h>

        #ifndef SYS_memfd_create

            #define __NR_memfd_create 319
            #define SYS_memfd_create __NR_memfd_create

int memfd_create(const char* name, unsigned int flags) {
    return syscall(SYS_memfd_create, name, flags);
}

        #endif
    #endif
#endif
