#include <poll.h>
#include <unistd.h>

#include <cstdlib>

#include "depthai/depthai.hpp"

int main(int argc, char* argv[]) {
    if(argc < 3) {
        return EXIT_FAILURE;
    }
    const int readyFd = std::atoi(argv[1]);
    const int goFd = std::atoi(argv[2]);

    for(int index = 0; index < 11; ++index) {
        dai::Pipeline pipeline(false);
        pipeline.start();
        pipeline.stop();
    }

    const char ready = 'r';
    if(::write(readyFd, &ready, 1) != 1) {
        return EXIT_FAILURE;
    }
    pollfd go{goFd, POLLIN, 0};
    ::poll(&go, 1, 30000);
    char goByte = 0;
    if(::read(goFd, &goByte, 1) != 1) {
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
