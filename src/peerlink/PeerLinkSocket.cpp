#include "peerlink/PeerLinkSocket.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <utility>

namespace dai {
namespace peerlink {

namespace socketdetail {

#ifdef MSG_NOSIGNAL
constexpr int SEND_FLAGS = MSG_NOSIGNAL;
#else
constexpr int SEND_FLAGS = 0;
#endif

std::string errnoString(int error) {
    return std::strerror(error);
}

timeval toTimeval(std::chrono::milliseconds timeout) {
    timeval tv{};
    tv.tv_sec = static_cast<decltype(tv.tv_sec)>(timeout.count() / 1000);
    tv.tv_usec = static_cast<decltype(tv.tv_usec)>((timeout.count() % 1000) * 1000);
    return tv;
}

bool setBlocking(int fd, bool blocking) {
    const int flags = fcntl(fd, F_GETFL, 0);
    if(flags < 0) return false;
    const int newFlags = blocking ? (flags & ~O_NONBLOCK) : (flags | O_NONBLOCK);
    return fcntl(fd, F_SETFL, newFlags) == 0;
}

// Connects one resolved address, waiting at most `timeout`
int connectAddress(const addrinfo* address, std::chrono::steady_clock::time_point deadline, std::string& error) {
    const int fd = ::socket(address->ai_family, address->ai_socktype, address->ai_protocol);
    if(fd < 0) {
        error = "socket(): " + errnoString(errno);
        return -1;
    }
#ifdef FD_CLOEXEC
    fcntl(fd, F_SETFD, FD_CLOEXEC);
#endif
    if(!setBlocking(fd, false)) {
        error = "fcntl(): " + errnoString(errno);
        ::close(fd);
        return -1;
    }
    int ret = ::connect(fd, address->ai_addr, address->ai_addrlen);
    if(ret != 0 && errno != EINPROGRESS) {
        error = "connect(): " + errnoString(errno);
        ::close(fd);
        return -1;
    }
    if(ret != 0) {
        pollfd pfd{fd, POLLOUT, 0};
        do {
            const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - std::chrono::steady_clock::now());
            ret = ::poll(&pfd, 1, static_cast<int>(std::max<std::chrono::milliseconds::rep>(remaining.count(), 0)));
        } while(ret < 0 && errno == EINTR);
        if(ret == 0) {
            error = "connect timed out";
            ::close(fd);
            return -1;
        }
        if(ret < 0) {
            error = "poll(): " + errnoString(errno);
            ::close(fd);
            return -1;
        }
        int socketError = 0;
        socklen_t length = sizeof(socketError);
        if(::getsockopt(fd, SOL_SOCKET, SO_ERROR, &socketError, &length) != 0 || socketError != 0) {
            error = "connect(): " + errnoString(socketError != 0 ? socketError : errno);
            ::close(fd);
            return -1;
        }
    }
    if(!setBlocking(fd, true)) {
        error = "fcntl(): " + errnoString(errno);
        ::close(fd);
        return -1;
    }
    return fd;
}

}  // namespace socketdetail

Socket::~Socket() {
    close();
}

Socket::Socket(Socket&& other) noexcept : fd(std::exchange(other.fd, -1)) {}

Socket& Socket::operator=(Socket&& other) noexcept {
    if(this != &other) {
        close();
        fd = std::exchange(other.fd, -1);
    }
    return *this;
}

void Socket::close() {
    if(fd >= 0) {
        ::close(fd);
        fd = -1;
    }
}

void Socket::shutdown() const {
    if(fd >= 0) {
        ::shutdown(fd, SHUT_RDWR);
    }
}

const char* toString(IoResult result) {
    switch(result) {
        case IoResult::OK:
            return "ok";
        case IoResult::CLOSED:
            return "connection closed by peer";
        case IoResult::TIMEOUT:
            return "timed out";
        case IoResult::FAILED:
            return "socket error";
    }
    return "unknown";
}

Socket connectTcp(const std::string& host, std::uint16_t port, std::chrono::milliseconds timeout, std::string* error, bool numericHost) {
    // One deadline for all resolved addresses
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    addrinfo hints{};
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_flags = AI_NUMERICSERV | (numericHost ? AI_NUMERICHOST : 0);
    addrinfo* addresses = nullptr;
    const auto portString = std::to_string(port);
    const int ret = ::getaddrinfo(host.c_str(), portString.c_str(), &hints, &addresses);
    if(ret != 0) {
        if(error) *error = std::string("getaddrinfo(): ") + gai_strerror(ret);
        return Socket();
    }
    std::string lastError = "no address";
    for(const addrinfo* address = addresses; address != nullptr; address = address->ai_next) {
        const int fd = socketdetail::connectAddress(address, deadline, lastError);
        if(fd >= 0) {
            ::freeaddrinfo(addresses);
            return Socket(fd);
        }
    }
    ::freeaddrinfo(addresses);
    if(error) *error = lastError;
    return Socket();
}

Socket listenTcp(const std::string& bindAddress, std::uint16_t port, int backlog) {
    addrinfo hints{};
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_flags = AI_PASSIVE | AI_NUMERICSERV | AI_NUMERICHOST;
    addrinfo* addresses = nullptr;
    const auto portString = std::to_string(port);
    const int ret = ::getaddrinfo(bindAddress.empty() ? nullptr : bindAddress.c_str(), portString.c_str(), &hints, &addresses);
    if(ret != 0) {
        throw std::runtime_error(std::string("Peer-link listener: getaddrinfo(): ") + gai_strerror(ret));
    }
    std::string lastError = "no address";
    for(const addrinfo* address = addresses; address != nullptr; address = address->ai_next) {
        Socket socket(::socket(address->ai_family, address->ai_socktype, address->ai_protocol));
        if(!socket.valid()) {
            lastError = "socket(): " + socketdetail::errnoString(errno);
            continue;
        }
#ifdef FD_CLOEXEC
        fcntl(socket.get(), F_SETFD, FD_CLOEXEC);
#endif
        const int enable = 1;
        ::setsockopt(socket.get(), SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(enable));
        if(::bind(socket.get(), address->ai_addr, address->ai_addrlen) != 0) {
            lastError = "bind(): " + socketdetail::errnoString(errno);
            continue;
        }
        if(::listen(socket.get(), backlog) != 0) {
            lastError = "listen(): " + socketdetail::errnoString(errno);
            continue;
        }
        ::freeaddrinfo(addresses);
        return socket;
    }
    ::freeaddrinfo(addresses);
    throw std::runtime_error("Peer-link listener on " + bindAddress + ":" + portString + ": " + lastError);
}

std::uint16_t localPort(const Socket& socket) {
    sockaddr_storage address{};
    socklen_t length = sizeof(address);
    if(::getsockname(socket.get(), reinterpret_cast<sockaddr*>(&address), &length) != 0) {
        return 0;
    }
    if(address.ss_family == AF_INET) {
        return ntohs(reinterpret_cast<sockaddr_in*>(&address)->sin_port);
    }
    if(address.ss_family == AF_INET6) {
        return ntohs(reinterpret_cast<sockaddr_in6*>(&address)->sin6_port);
    }
    return 0;
}

void configureStreamSocket(const Socket& socket) {
    const int enable = 1;
    ::setsockopt(socket.get(), IPPROTO_TCP, TCP_NODELAY, &enable, sizeof(enable));
    ::setsockopt(socket.get(), SOL_SOCKET, SO_KEEPALIVE, &enable, sizeof(enable));
#ifdef SO_NOSIGPIPE
    ::setsockopt(socket.get(), SOL_SOCKET, SO_NOSIGPIPE, &enable, sizeof(enable));
#endif
}

void setSendTimeout(const Socket& socket, std::chrono::milliseconds timeout) {
    const auto tv = socketdetail::toTimeval(timeout);
    ::setsockopt(socket.get(), SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
}

void setReceiveTimeout(const Socket& socket, std::chrono::milliseconds timeout) {
    const auto tv = socketdetail::toTimeval(timeout);
    ::setsockopt(socket.get(), SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
}

bool isWritable(const Socket& socket) {
    pollfd pfd{socket.get(), POLLOUT, 0};
    int ready = 0;
    do {
        ready = ::poll(&pfd, 1, 0);
    } while(ready < 0 && errno == EINTR);
    return ready > 0 && (pfd.revents & POLLOUT) != 0;
}

IoResult sendAll(const Socket& socket, iovec* buffers, int count) {
    // Skip empty leading buffers, sendmsg() would return 0 for them
    while(count > 0 && buffers->iov_len == 0) {
        buffers++;
        count--;
    }
    while(count > 0) {
        msghdr message{};
        message.msg_iov = buffers;
        message.msg_iovlen = static_cast<decltype(message.msg_iovlen)>(count);
        const ssize_t sent = ::sendmsg(socket.get(), &message, socketdetail::SEND_FLAGS);
        if(sent < 0) {
            if(errno == EINTR) continue;
            if(errno == EAGAIN || errno == EWOULDBLOCK) return IoResult::TIMEOUT;
            if(errno == EPIPE || errno == ECONNRESET) return IoResult::CLOSED;
            return IoResult::FAILED;
        }
        auto remaining = static_cast<std::size_t>(sent);
        while(count > 0 && remaining >= buffers->iov_len) {
            remaining -= buffers->iov_len;
            buffers++;
            count--;
        }
        if(count > 0) {
            buffers->iov_base = static_cast<std::uint8_t*>(buffers->iov_base) + remaining;
            buffers->iov_len -= remaining;
        }
    }
    return IoResult::OK;
}

IoResult receiveAll(const Socket& socket, void* data, std::size_t size) {
    auto* out = static_cast<std::uint8_t*>(data);
    while(size > 0) {
        const ssize_t received = ::recv(socket.get(), out, size, 0);
        if(received == 0) return IoResult::CLOSED;
        if(received < 0) {
            if(errno == EINTR) continue;
            if(errno == EAGAIN || errno == EWOULDBLOCK) return IoResult::TIMEOUT;
            if(errno == ECONNRESET) return IoResult::CLOSED;
            return IoResult::FAILED;
        }
        out += received;
        size -= static_cast<std::size_t>(received);
    }
    return IoResult::OK;
}

IoResult receiveAllUntil(const Socket& socket, void* data, std::size_t size, std::chrono::steady_clock::time_point deadline) {
    auto* out = static_cast<std::uint8_t*>(data);
    while(size > 0) {
        const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - std::chrono::steady_clock::now());
        if(remaining.count() <= 0) return IoResult::TIMEOUT;
        pollfd pfd{socket.get(), POLLIN, 0};
        const int ready = ::poll(&pfd, 1, static_cast<int>(remaining.count()));
        if(ready < 0) {
            if(errno == EINTR) continue;
            return IoResult::FAILED;
        }
        if(ready == 0) return IoResult::TIMEOUT;
        const ssize_t received = ::recv(socket.get(), out, size, 0);
        if(received == 0) return IoResult::CLOSED;
        if(received < 0) {
            if(errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) continue;
            if(errno == ECONNRESET) return IoResult::CLOSED;
            return IoResult::FAILED;
        }
        out += received;
        size -= static_cast<std::size_t>(received);
    }
    return IoResult::OK;
}

}  // namespace peerlink
}  // namespace dai
