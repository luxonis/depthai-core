#pragma once

// POSIX socket helpers for the peer-link transport. Built on Linux and macOS only (the firmware and the
// on-host tests use them); the host library on Windows never opens a peer-link socket.

#include <sys/uio.h>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>

namespace dai {
namespace peerlink {

/// Owns a socket file descriptor
class Socket {
   public:
    Socket() = default;
    explicit Socket(int fd) : fd(fd) {}
    ~Socket();
    Socket(const Socket&) = delete;
    Socket& operator=(const Socket&) = delete;
    Socket(Socket&& other) noexcept;
    Socket& operator=(Socket&& other) noexcept;

    int get() const {
        return fd;
    }
    bool valid() const {
        return fd >= 0;
    }
    void close();
    /// Unblocks a send or receive that another thread is waiting in; the descriptor stays open
    void shutdown() const;

   private:
    int fd = -1;
};

enum class IoResult : std::uint8_t {
    OK,
    /// The peer closed the connection
    CLOSED,
    /// No progress within the socket's send or receive timeout
    TIMEOUT,
    FAILED,
};

const char* toString(IoResult result);

/**
 * Connects to host:port, waiting at most `timeout` (the name lookup excepted).
 * @param error Receives the reason when the connection fails
 * @param numericHost Accept only a numeric address, so no name lookup (which has no time limit) can run
 * @returns An invalid socket when the connection fails
 */
Socket connectTcp(const std::string& host, std::uint16_t port, std::chrono::milliseconds timeout, std::string* error = nullptr, bool numericHost = false);

/// Binds and listens on bindAddress:port with SO_REUSEADDR. @throws std::runtime_error on failure
Socket listenTcp(const std::string& bindAddress, std::uint16_t port, int backlog = 16);

/// The local port a socket is bound to
std::uint16_t localPort(const Socket& socket);

/// TCP_NODELAY, SO_KEEPALIVE and no SIGPIPE on send
void configureStreamSocket(const Socket& socket);
void setSendTimeout(const Socket& socket, std::chrono::milliseconds timeout);
void setReceiveTimeout(const Socket& socket, std::chrono::milliseconds timeout);

/// True when a send would not block (the socket has room in its send buffer)
bool isWritable(const Socket& socket);

/// Sends all bytes of the buffers. The socket's send timeout bounds the time without progress.
IoResult sendAll(const Socket& socket, iovec* buffers, int count);
/// Receives exactly `size` bytes. The socket's receive timeout bounds the time without progress.
IoResult receiveAll(const Socket& socket, void* data, std::size_t size);
/// Receives exactly `size` bytes before `deadline`, however slowly the peer sends them
IoResult receiveAllUntil(const Socket& socket, void* data, std::size_t size, std::chrono::steady_clock::time_point deadline);

}  // namespace peerlink
}  // namespace dai
