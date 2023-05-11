#include "utility/XLinkGlobalProfilingLogger.hpp"

#include <XLink/XLink.h>

#include "Logging.hpp"
#include "depthai/utility/Initialization.hpp"

using namespace std;
using namespace std::chrono;

namespace dai {

XLinkGlobalProfilingLogger::XLinkGlobalProfilingLogger() {}

void XLinkGlobalProfilingLogger::enable(bool en) {
    running = false;
    if(thr.joinable()) thr.join();

    if(en) {
        running = true;
        XLinkProfStart();
        thr = std::thread([this]() {
            XLinkProf_t lastProf = {};
            while(running) {
                XLinkProf_t prof;
                XLinkGetGlobalProfilingData(&prof);

                const auto rateSnapshot = rate.load();
                const float w = (prof.totalWriteBytes - lastProf.totalWriteBytes) / rateSnapshot;
                const float r = (prof.totalReadBytes - lastProf.totalReadBytes) / rateSnapshot;

                logger::debug("Profiling global write speed: {:.2f} MiB/s, read speed: {:.2f} MiB/s, total written: {:.2f} MiB, read: {:.2f} MiB",
                              w / 1024.0f / 1024.0f,
                              r / 1024.0f / 1024.0f,
                              prof.totalWriteBytes / 1024.0f / 1024.0f,
                              prof.totalReadBytes / 1024.0f / 1024.0f);

                lastProf = std::move(prof);
                this_thread::sleep_for(duration<float>(1) / rateSnapshot);
            }
        });
    }
}
void XLinkGlobalProfilingLogger::setRate(float rate) {
    this->rate = rate;
}

float XLinkGlobalProfilingLogger::getRate() {
    return rate;
}
XLinkGlobalProfilingLogger::~XLinkGlobalProfilingLogger() {
    enable(false);
}

XLinkGlobalProfilingLogger& XLinkGlobalProfilingLogger::getInstance() {
    static XLinkGlobalProfilingLogger instance;  // Guaranteed to be destroyed, instantiated on first use.
    return instance;
}

}  // namespace dai
