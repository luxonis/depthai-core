#pragma once

#include <spdlog/details/null_mutex.h>
#include <spdlog/sinks/base_sink.h>

#include <mutex>
#include <ostream>

#include "SpdlogDaiConversion.hpp"
#include "depthai/log/LogMessage.hpp"

namespace dai {
namespace gate {

template<typename Mutex>
class CallbackSink final : public spdlog::sinks::base_sink<Mutex>
{
public:

    using callback_t = std::function<void(dai::LogMessage daiMsg)>;

    explicit CallbackSink(callback_t cb) : callback(cb) {}
    CallbackSink(const CallbackSink &) = delete;
    CallbackSink &operator=(const CallbackSink &) = delete;

protected:
    void sink_it_(const spdlog::details::log_msg &msg) override
    {
        callback(spdlogMsgToDaiLogMessage(msg));
    }

    void flush_() override
    {
        // ignore, no flush
        // TODO(Themarpe) - create a callback for flush
        // and test if flush is called after all async messages arrive
    }

    callback_t callback;
};

using CallbackSink_mt = CallbackSink<std::mutex>;
using CallbackSink_st = CallbackSink<spdlog::details::null_mutex>;

} // namespace gate
} // namespace dai