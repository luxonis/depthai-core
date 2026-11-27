#include <unistd.h>

#include <chrono>
#include <cstdlib>
#include <thread>

#include "depthai/depthai.hpp"

// Child of telemetry_exit_test: it queues enough telemetry events that an upload runs at exit, writes one byte to the
// pipe file descriptor argv[1] and returns from main. The static destructors run the telemetry shutdown, and the test
// measures the time from the byte to the end of the process.
int main(int argc, char* argv[]) {
    if(argc < 2) {
        return EXIT_FAILURE;
    }
    for(int index = 0; index < 11; ++index) {  // 22 pipeline start and stop events, more than the flush threshold (20)
        dai::Pipeline pipeline(false);
        pipeline.start();
        pipeline.stop();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));  // The worker starts the upload
    const char exiting = 'x';
    if(::write(std::atoi(argv[1]), &exiting, 1) != 1) {
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
