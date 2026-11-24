#ifndef FASTCVUTILS_HPP
#define FASTCVUTILS_HPP

#include <mutex>
#include <utility>

#include <fastcv/fastcv.h>
#include <fastcv/fastcvExt.h>

void fcvInit(fcvOperationMode operMode);
std::lock_guard<std::mutex> getFcvLock();

template <typename Func, typename... Args>
auto fcvRun(Func f, Args&&... args) -> decltype(f(std::forward<Args>(args)...)) {
    auto lock = getFcvLock();
    return f(std::forward<Args>(args)...);
}

#endif
