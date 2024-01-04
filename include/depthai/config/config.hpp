#pragma once

// Build specific settings overwrite
#ifdef DEPTHAI_TARGET_CORE
    #ifndef DEPTHAI_TARGET_OPENCV
        #undef DEPTHAI_HAVE_OPENCV_SUPPORT
    #endif
#endif
