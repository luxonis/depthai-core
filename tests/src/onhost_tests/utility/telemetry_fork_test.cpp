#include <catch2/catch_all.hpp>

#if defined(__unix__) || defined(__APPLE__)
    #include <sys/wait.h>
    #include <unistd.h>

    #include <chrono>
    #include <csignal>
    #include <cstdlib>
    #include <filesystem>
    #include <thread>

    #include "depthai/depthai.hpp"
    #include "utility/Platform.hpp"
#endif

TEST_CASE("telemetry remains usable in a fork child") {
#if defined(__unix__) || defined(__APPLE__)
    const auto cacheDir = dai::platform::getTempPath();
    REQUIRE(::setenv("DEPTHAI_CACHE_DIR", cacheDir.c_str(), 1) == 0);
    REQUIRE(::setenv("DEPTHAI_TELEMETRY_URL", "http://127.0.0.1:1", 1) == 0);
    REQUIRE(::setenv("DEPTHAI_TELEMETRY_API_KEY", "telemetry-fork-test", 1) == 0);

    REQUIRE(dai::initialize());
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    const auto childPid = ::fork();
    REQUIRE(childPid >= 0);

    if(childPid == 0) {
        dai::Pipeline pipeline(false);
        pipeline.start();
        pipeline.stop();
        ::_exit(EXIT_SUCCESS);
    }

    int status = 0;
    bool childExited = false;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while(std::chrono::steady_clock::now() < deadline) {
        const auto waitResult = ::waitpid(childPid, &status, WNOHANG);
        if(waitResult == childPid) {
            childExited = true;
            break;
        }
        REQUIRE(waitResult >= 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if(!childExited) {
        REQUIRE(::kill(childPid, SIGKILL) == 0);
        REQUIRE(::waitpid(childPid, &status, 0) == childPid);
    }

    std::error_code ec;
    std::filesystem::remove_all(cacheDir, ec);

    REQUIRE(childExited);
    REQUIRE(WIFEXITED(status));
    REQUIRE(WEXITSTATUS(status) == EXIT_SUCCESS);
#else
    SKIP("fork is only available on POSIX platforms");
#endif
}
