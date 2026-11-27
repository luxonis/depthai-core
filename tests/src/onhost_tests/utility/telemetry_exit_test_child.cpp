#include <poll.h>
#include <unistd.h>

#include <cstdlib>

#include "depthai/depthai.hpp"

// Child of telemetry_exit_test: it queues enough telemetry events that an upload starts, writes one byte to the pipe
// file descriptor argv[1], waits for one byte on argv[2] (the parent saw the upload connect) and returns from main.
// The static destructors run the telemetry shutdown, and the test measures the time to the end of the process.
int main(int argc, char* argv[]) {
    if(argc < 3) {
        return EXIT_FAILURE;
    }
    const int readyFd = std::atoi(argv[1]);
    const int goFd = std::atoi(argv[2]);

    for(int index = 0; index < 11; ++index) {  // 22 pipeline start and stop events, more than the flush threshold (20)
        dai::Pipeline pipeline(false);
        pipeline.start();
        pipeline.stop();
    }

    const char ready = 'r';
    if(::write(readyFd, &ready, 1) != 1) {
        return EXIT_FAILURE;
    }
    pollfd go{goFd, POLLIN, 0};
    ::poll(&go, 1, 30000);  // End of file (the parent ended) or a timeout also lets the child exit
    char goByte = 0;
    if(::read(goFd, &goByte, 1) != 1) {
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
