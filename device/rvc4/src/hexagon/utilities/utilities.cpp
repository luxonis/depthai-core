// The C function dai_device_dsp_status_name of the DSP offload (vendor/dai_device_dsp.h). AEEStdErr.h is a header of the
// Hexagon SDK: the StereoDepthDsp library compiles this file, so that public code (hexagon::errorToString in
// utilities.hpp) gets the names without the SDK.
#include <cstdio>

#include "AEEStdErr.h"
#include "vendor/dai_device_dsp.h"

extern "C" const char* dai_device_dsp_status_name(int32_t errorCode) {
    switch (errorCode) {
        case AEE_SUCCESS: return "AEE_SUCCESS";
        case AEE_EUNKNOWN: return "AEE_EUNKNOWN";
        case AEE_EFAILED: return "AEE_EFAILED";
        case AEE_ENOMEMORY: return "AEE_ENOMEMORY";
        case AEE_ECLASSNOTSUPPORT: return "AEE_ECLASSNOTSUPPORT";
        case AEE_EVERSIONNOTSUPPORT: return "AEE_EVERSIONNOTSUPPORT";
        case AEE_EALREADYLOADED: return "AEE_EALREADYLOADED";
        case AEE_EUNABLETOLOAD: return "AEE_EUNABLETOLOAD";
        case AEE_EUNABLETOUNLOAD: return "AEE_EUNABLETOUNLOAD";
        case AEE_EALARMPENDING: return "AEE_EALARMPENDING";
        case AEE_EINVALIDTIME: return "AEE_EINVALIDTIME";
        case AEE_EBADCLASS: return "AEE_EBADCLASS";
        case AEE_EBADMETRIC: return "AEE_EBADMETRIC";
        case AEE_EEXPIRED: return "AEE_EEXPIRED";
        case AEE_EBADSTATE: return "AEE_EBADSTATE";
        case AEE_EBADPARM: return "AEE_EBADPARM";
        case AEE_ESCHEMENOTSUPPORTED: return "AEE_ESCHEMENOTSUPPORTED";
        case AEE_EBADITEM: return "AEE_EBADITEM";
        case AEE_EINVALIDFORMAT: return "AEE_EINVALIDFORMAT";
        case AEE_EINCOMPLETEITEM: return "AEE_EINCOMPLETEITEM";
        case AEE_ENOPERSISTMEMORY: return "AEE_ENOPERSISTMEMORY";
        case AEE_EUNSUPPORTED: return "AEE_EUNSUPPORTED";
        case AEE_EPRIVLEVEL: return "AEE_EPRIVLEVEL";
        case AEE_ERESOURCENOTFOUND: return "AEE_ERESOURCENOTFOUND";
        case AEE_EREENTERED: return "AEE_EREENTERED";
        case AEE_EBADTASK: return "AEE_EBADTASK";
        case AEE_EALLOCATED: return "AEE_EALLOCATED";
        case AEE_EALREADY: return "AEE_EALREADY";
        case AEE_EADSAUTHBAD: return "AEE_EADSAUTHBAD";
        case AEE_ENEEDSERVICEPROG: return "AEE_ENEEDSERVICEPROG";
        case AEE_EMEMPTR: return "AEE_EMEMPTR";
        case AEE_EHEAP: return "AEE_EHEAP";
        case AEE_EIDLE: return "AEE_EIDLE";
        case AEE_EITEMBUSY: return "AEE_EITEMBUSY";
        case AEE_EBADSID: return "AEE_EBADSID";
        case AEE_ENOTYPE: return "AEE_ENOTYPE";
        case AEE_ENEEDMORE: return "AEE_ENEEDMORE";
        case AEE_EADSCAPS: return "AEE_EADSCAPS";
        case AEE_EBADSHUTDOWN: return "AEE_EBADSHUTDOWN";
        case AEE_EBUFFERTOOSMALL: return "AEE_EBUFFERTOOSMALL";
        case AEE_EACKPENDING: return "AEE_EACKPENDING";
        case AEE_ENOTOWNER: return "AEE_ENOTOWNER";
        case AEE_EINVALIDITEM: return "AEE_EINVALIDITEM";
        case AEE_ENOTALLOWED: return "AEE_ENOTALLOWED";
        case AEE_EINVHANDLE: return "AEE_EINVHANDLE";
        // case AEE_EBADHANDLE: duplicate of AEE_EINVHANDLE
        case AEE_ENOMORE: return "AEE_ENOMORE";
        case AEE_ECPUEXCEPTION: return "AEE_ECPUEXCEPTION";
        case AEE_EREADONLY: return "AEE_EREADONLY";
        case AEE_ERPC: return "AEE_ERPC";
        case AEE_EFILE: return "AEE_EFILE";
        case AEE_ENOSUCH: return "AEE_ENOSUCH";
        case AEE_EINTERRUPTED: return "AEE_EINTERRUPTED";
        case AEE_EWOULDBLOCK: return "AEE_EWOULDBLOCK";
        case AEE_EINVALIDMSG: return "AEE_EINVALIDMSG";
        case AEE_EINVALIDTHREAD: return "AEE_EINVALIDTHREAD";
        case AEE_EINVALIDPROCESS: return "AEE_EINVALIDPROCESS";
        case AEE_EINVALIDFILENAME: return "AEE_EINVALIDFILENAME";
        case AEE_EINVALIDDIGESTSIZE: return "AEE_EINVALIDDIGESTSIZE";
        case AEE_EINVALIDSEGS: return "AEE_EINVALIDSEGS";
        case AEE_EINVALIDSIGNATURE: return "AEE_EINVALIDSIGNATURE";
        case AEE_EINVALIDDOMAIN: return "AEE_EINVALIDDOMAIN";
        case AEE_EINVALIDFD: return "AEE_EINVALIDFD";
        case AEE_EINVALIDDEVICE: return "AEE_EINVALIDDEVICE";
        case AEE_EINVALIDMODE: return "AEE_EINVALIDMODE";
        case AEE_EINVALIDPROCNAME: return "AEE_EINVALIDPROCNAME";
        case AEE_ENOSUCHMOD: return "AEE_ENOSUCHMOD";
        case AEE_ENOSUCHINSTANCE: return "AEE_ENOSUCHINSTANCE";
        case AEE_ENOSUCHTHREAD: return "AEE_ENOSUCHTHREAD";
        case AEE_ENOSUCHPROCESS: return "AEE_ENOSUCHPROCESS";
        case AEE_ENOSUCHSYMBOL: return "AEE_ENOSUCHSYMBOL";
        case AEE_ENOSUCHDEVICE: return "AEE_ENOSUCHDEVICE";
        case AEE_ENOSUCHPROP: return "AEE_ENOSUCHPROP";
        case AEE_ENOSUCHFILE: return "AEE_ENOSUCHFILE";
        case AEE_ENOSUCHHANDLE: return "AEE_ENOSUCHHANDLE";
        case AEE_ENOSUCHSTREAM: return "AEE_ENOSUCHSTREAM";
        case AEE_ENOSUCHMAP: return "AEE_ENOSUCHMAP";
        case AEE_ENOSUCHREGISTER: return "AEE_ENOSUCHREGISTER";
        case AEE_ENOSUCHCLIENT: return "AEE_ENOSUCHCLIENT";
        case AEE_EBADDOMAIN: return "AEE_EBADDOMAIN";
        case AEE_EBADOFFSET: return "AEE_EBADOFFSET";
        case AEE_EBADSIZE: return "AEE_EBADSIZE";
        case AEE_EBADPERMS: return "AEE_EBADPERMS";
        case AEE_EBADFD: return "AEE_EBADFD";
        case AEE_EBADPID: return "AEE_EBADPID";
        case AEE_EBADTID: return "AEE_EBADTID";
        case AEE_EBADELF: return "AEE_EBADELF";
        case AEE_EBADASID: return "AEE_EBADASID";
        case AEE_EBADCONTEXT: return "AEE_EBADCONTEXT";
        case AEE_EBADMEMALIGN: return "AEE_EBADMEMALIGN";
        case AEE_EIOCTL: return "AEE_EIOCTL";
        case AEE_EFOPEN: return "AEE_EFOPEN";
        case AEE_EFGETS: return "AEE_EFGETS";
        case AEE_EFFLUSH: return "AEE_EFFLUSH";
        case AEE_EFCLOSE: return "AEE_EFCLOSE";
        case AEE_EEOF: return "AEE_EEOF";
        case AEE_EFREAD: return "AEE_EFREAD";
        case AEE_EFWRITE: return "AEE_EFWRITE";
        case AEE_EFGETPOS: return "AEE_EFGETPOS";
        case AEE_EFSETPOS: return "AEE_EFSETPOS";
        case AEE_EFTELL: return "AEE_EFTELL";
        case AEE_EFSEEK: return "AEE_EFSEEK";
        case AEE_EFLEN: return "AEE_EFLEN";
        case AEE_EGETENV: return "AEE_EGETENV";
        case AEE_ESETENV: return "AEE_ESETENV";
        case AEE_EMMAP: return "AEE_EMMAP";
        case AEE_EIONMAP: return "AEE_EIONMAP";
        case AEE_EIONALLOC: return "AEE_EIONALLOC";
        case AEE_ENORPCMEMORY: return "AEE_ENORPCMEMORY";
        case AEE_ENOROOTOFTRUST: return "AEE_ENOROOTOFTRUST";
        case AEE_ENOTLOCKED: return "AEE_ENOTLOCKED";
        case AEE_ENOTINITIALIZED: return "AEE_ENOTINITIALIZED";
        case AEE_EUNSUPPORTEDAPI: return "AEE_EUNSUPPORTEDAPI";
        case AEE_EUNPACK: return "AEE_EUNPACK";
        case AEE_EPOLL: return "AEE_EPOLL";
        case AEE_EEVENTREAD: return "AEE_EEVENTREAD";
        case AEE_EMAXBUFS: return "AEE_EMAXBUFS";
        case AEE_EINVARGS: return "AEE_EINVARGS";
        case AEE_ECONNREFUSED: return "AEE_ECONNREFUSED";
        // case AEE_ENOSESSION: duplicate of AEE_ECONNREFUSED
        // You can add more guards for undefined constants as needed
        default: {
            // The same text as before the C interface; one buffer for each thread
            thread_local char unknown[48];
            std::snprintf(unknown, sizeof(unknown), "Unknown error code: %d", static_cast<int>(errorCode));
            return unknown;
        }
    }
}
