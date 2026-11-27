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

    int readyPipe[2] = {-1, -1};  // Child to parent: the events are queued
    int goPipe[2] = {-1, -1};     // Parent to child: return from main
    REQUIRE(::pipe(readyPipe) == 0);
    cleanup.fds.insert(cleanup.fds.end(), {readyPipe[0], readyPipe[1]});
    REQUIRE(::pipe(goPipe) == 0);
    cleanup.fds.insert(cleanup.fds.end(), {goPipe[0], goPipe[1]});

    cleanup.cacheDir = dai::platform::getTempPath();
    REQUIRE(::setenv("DEPTHAI_CACHE_DIR", cleanup.cacheDir.c_str(), 1) == 0);
    REQUIRE(::setenv("DEPTHAI_TELEMETRY_URL", url.c_str(), 1) == 0);
    REQUIRE(::setenv("DEPTHAI_TELEMETRY_API_KEY", "telemetry-exit-test", 1) == 0);
    REQUIRE(::unsetenv("DEPTHAI_TELEMETRY") == 0);

    const auto readyWriteFd = std::to_string(readyPipe[1]);
    const auto goReadFd = std::to_string(goPipe[0]);
    const auto childPid = ::fork();
    REQUIRE(childPid >= 0);
    if(childPid == 0) {
        // A new program: the static destructors of a forked copy of this process are not safe to run
        ::execl(DEPTHAI_TELEMETRY_EXIT_TEST_CHILD_PATH,
                DEPTHAI_TELEMETRY_EXIT_TEST_CHILD_PATH,
                readyWriteFd.c_str(),
                goReadFd.c_str(),
                static_cast<char*>(nullptr));
        ::_exit(127);
    }
    cleanup.pid = childPid;
    // Close the ends that only the child uses: a child that ends early gives end of file
    ::close(readyPipe[1]);
    ::close(goPipe[0]);
    cleanup.fds[2] = -1;
    cleanup.fds[3] = -1;

    const auto pollIn = [](int fd, int timeoutMs) {
        pollfd request{fd, POLLIN, 0};
        int result = -1;
        do {
            result = ::poll(&request, 1, timeoutMs);
        } while(result < 0 && errno == EINTR);
        return result;
    };

    REQUIRE(pollIn(readyPipe[0], 30000) == 1);
    char readyByte = 0;
    REQUIRE(::read(readyPipe[0], &readyByte, 1) == 1);

    // The upload connects to the server: it is in progress when the exit starts
    REQUIRE(pollIn(server, 10000) == 1);

    const char goByte = 'g';
    REQUIRE(::write(goPipe[1], &goByte, 1) == 1);
    const auto exitStart = std::chrono::steady_clock::now();

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
