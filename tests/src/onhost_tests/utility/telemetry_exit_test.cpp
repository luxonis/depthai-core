#include <catch2/catch_all.hpp>

#if defined(__unix__) || defined(__APPLE__)
    #include <arpa/inet.h>
    #include <fcntl.h>
    #include <netinet/in.h>
    #include <poll.h>
    #include <sys/socket.h>
    #include <sys/wait.h>
    #include <unistd.h>

    #include <cerrno>
    #include <chrono>
    #include <csignal>
    #include <cstdlib>
    #include <filesystem>
    #include <optional>
    #include <string>
    #include <thread>
    #include <vector>

    #include "utility/Platform.hpp"

namespace {

// Kills and reaps the child, and closes the file descriptors, also when a REQUIRE fails
struct ChildCleanup {
    pid_t pid = -1;
    std::vector<int> fds;
    std::filesystem::path cacheDir;

    ~ChildCleanup() {
        if(pid > 0) {
            ::kill(pid, SIGKILL);
            while(::waitpid(pid, nullptr, 0) < 0 && errno == EINTR) {
            }
        }
        for(const int fd : fds) {
            if(fd >= 0) {
                ::close(fd);
            }
        }
        std::error_code ec;
        std::filesystem::remove_all(cacheDir, ec);
    }
};

}  // namespace
#endif

// A telemetry server that does not answer must not hold the process exit for the full upload timeout (10 s for each
// upload): the HIL test wrapper kills an example that does not exit 7 s after SIGINT.
TEST_CASE("telemetry does not hold the process exit when the server does not answer") {
#if defined(__unix__) || defined(__APPLE__)
    ChildCleanup cleanup;

    // The kernel completes the TCP handshake for the listen backlog, but nothing reads the request or answers it
    const int server = ::socket(AF_INET, SOCK_STREAM, 0);
    REQUIRE(server >= 0);
    cleanup.fds.push_back(server);
    REQUIRE(::fcntl(server, F_SETFD, FD_CLOEXEC) == 0);
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    address.sin_port = 0;
    REQUIRE(::bind(server, reinterpret_cast<sockaddr*>(&address), sizeof(address)) == 0);
    REQUIRE(::listen(server, 128) == 0);
    socklen_t addressLength = sizeof(address);
    REQUIRE(::getsockname(server, reinterpret_cast<sockaddr*>(&address), &addressLength) == 0);
    const auto url = "http://127.0.0.1:" + std::to_string(ntohs(address.sin_port));

    int exitingPipe[2] = {-1, -1};
    REQUIRE(::pipe(exitingPipe) == 0);
    cleanup.fds.push_back(exitingPipe[0]);
    cleanup.fds.push_back(exitingPipe[1]);

    cleanup.cacheDir = dai::platform::getTempPath();
    REQUIRE(::setenv("DEPTHAI_CACHE_DIR", cleanup.cacheDir.c_str(), 1) == 0);
    REQUIRE(::setenv("DEPTHAI_TELEMETRY_URL", url.c_str(), 1) == 0);
    REQUIRE(::setenv("DEPTHAI_TELEMETRY_API_KEY", "telemetry-exit-test", 1) == 0);
    REQUIRE(::unsetenv("DEPTHAI_TELEMETRY") == 0);

    const auto pipeWriteFd = std::to_string(exitingPipe[1]);
    const auto childPid = ::fork();
    REQUIRE(childPid >= 0);
    if(childPid == 0) {
        // A new program: the static destructors of a forked copy of this process are not safe to run
        ::execl(DEPTHAI_TELEMETRY_EXIT_TEST_CHILD_PATH, DEPTHAI_TELEMETRY_EXIT_TEST_CHILD_PATH, pipeWriteFd.c_str(), static_cast<char*>(nullptr));
        ::_exit(127);
    }
    cleanup.pid = childPid;
    ::close(exitingPipe[1]);  // Only the child writes: a child that ends early gives end of file
    cleanup.fds.back() = -1;

    // The child writes one byte when it returns from main; the exit time starts there
    pollfd exiting{exitingPipe[0], POLLIN, 0};
    int pollResult = -1;
    do {
        pollResult = ::poll(&exiting, 1, 30000);
    } while(pollResult < 0 && errno == EINTR);
    REQUIRE(pollResult == 1);
    char exitingByte = 0;
    REQUIRE(::read(exitingPipe[0], &exitingByte, 1) == 1);
    const auto exitStart = std::chrono::steady_clock::now();

    // The child connected to the server: an upload is in progress when the exit starts
    pollfd connection{server, POLLIN, 0};
    REQUIRE(::poll(&connection, 1, 0) == 1);

    int status = 0;
    std::optional<std::chrono::steady_clock::duration> exitTime;
    while(std::chrono::steady_clock::now() - exitStart < std::chrono::seconds(30)) {
        const auto waitResult = ::waitpid(childPid, &status, WNOHANG);
        if(waitResult == childPid) {
            exitTime = std::chrono::steady_clock::now() - exitStart;
            cleanup.pid = -1;
            break;
        }
        REQUIRE((waitResult == 0 || errno == EINTR));
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    REQUIRE(exitTime.has_value());
    INFO("exit took " << std::chrono::duration_cast<std::chrono::milliseconds>(*exitTime).count() << " ms, wait status " << status);
    REQUIRE(WIFEXITED(status));
    REQUIRE(WEXITSTATUS(status) == EXIT_SUCCESS);
    // Before the fix: about 20 s (the upload in progress plus the final flush, 10 s each)
    REQUIRE(*exitTime < std::chrono::seconds(5));
#else
    SKIP("fork is only available on POSIX platforms");
#endif
}
